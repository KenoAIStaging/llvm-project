/* Copyright libuv project contributors. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/* Measurement probe, not a correctness test: determine how large a single
 * overlapped stream write submission may be on 64-bit Windows, what the
 * submission itself costs (page probe-and-lock time), and how much
 * throughput is lost by keeping only one overlapped send outstanding on a
 * TCP socket compared with two.
 *
 * The probe deliberately bypasses libuv's write path and talks to the OS
 * directly (WSASend / WriteFile with an event-based OVERLAPPED), so the
 * numbers describe the operating system, not libuv. Submission failures at
 * a given size are recorded and printed, not asserted; grep the runner log
 * for lines starting with "#PROBE".
 *
 * The two-outstanding experiment also validates, via per-page markers keyed
 * to absolute stream offsets, whether data from two simultaneously
 * outstanding WSASend calls is placed on the TCP stream in submission
 * order.
 */

#include "uv.h"
#include "task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !defined(_WIN32) || !defined(_WIN64)

TEST_IMPL(probe_max_write) {
  RETURN_SKIP("probe_max_write measures 64-bit Windows behavior only.");
}

#else /* 64-bit Windows */

#define PROBE_MB (1024ULL * 1024ULL)
#define PROBE_RECV_WINDOW (64ULL * PROBE_MB)
#define PROBE_PAGE 4096ULL
#define PROBE_WAIT_MS 120000
#define PROBE_MULTI_CHUNKS 8

static LARGE_INTEGER probe_freq;

static const unsigned long long probe_ladder[] = {
  64 * PROBE_MB,
  256 * PROBE_MB,
  512 * PROBE_MB,
  1024 * PROBE_MB,
  1536 * PROBE_MB,
  2048 * PROBE_MB - 65536,   /* just below 2^31 */
  3072 * PROBE_MB,           /* above 2^31: exercises signed-32 handling */
  4096 * PROBE_MB - 65536    /* just below the WSABUF/DWORD 2^32 cap */
};
#define PROBE_NSIZES (sizeof(probe_ladder) / sizeof(probe_ladder[0]))

static double probe_ms(LARGE_INTEGER a, LARGE_INTEGER b) {
  return (double) (b.QuadPart - a.QuadPart) * 1000.0 /
         (double) probe_freq.QuadPart;
}

/* One marker byte at every page boundary, derived from the absolute stream
 * offset. Offsets that differ by any multiple of 256MB..8GB produce a
 * different marker, so misplaced/reordered chunks are detected.
 */
static unsigned char probe_marker(unsigned long long off) {
  return (unsigned char) (0xA5u ^
                          (unsigned) (off >> 12) ^
                          (unsigned) (off >> 19) ^
                          (unsigned) (off >> 26) ^
                          (unsigned) (off >> 33));
}

static void probe_fill(unsigned char* p,
                       unsigned long long stream_base,
                       unsigned long long len) {
  unsigned long long o;
  for (o = 0; o < len; o += PROBE_PAGE)
    p[o] = probe_marker(stream_base + o);
}

typedef struct {
  int is_pipe;
  SOCKET sock;               /* receive socket when !is_pipe */
  HANDLE pipe;               /* receive handle when is_pipe */
  unsigned char* window;     /* fixed receive-and-discard window */
  size_t window_size;
  unsigned long long received;
  unsigned long long mismatches;
  unsigned long long first_bad_off;
  LARGE_INTEGER t_done;
} probe_recv_ctx_t;

static void probe_recv_thread(void* arg) {
  probe_recv_ctx_t* c = (probe_recv_ctx_t*) arg;

  for (;;) {
    size_t n;
    unsigned long long first;
    unsigned long long p;

    if (c->is_pipe) {
      DWORD got = 0;
      if (!ReadFile(c->pipe, c->window, (DWORD) c->window_size, &got, NULL))
        break;  /* ERROR_BROKEN_PIPE == EOF */
      if (got == 0)
        break;
      n = got;
    } else {
      int r = recv(c->sock, (char*) c->window, (int) c->window_size, 0);
      if (r <= 0)
        break;
      n = (size_t) r;
    }

    /* Verify one marker byte per page against the absolute stream offset. */
    first = (c->received + PROBE_PAGE - 1) & ~(PROBE_PAGE - 1);
    for (p = first; p < c->received + n; p += PROBE_PAGE) {
      unsigned char got_b = c->window[(size_t) (p - c->received)];
      unsigned char want_b = probe_marker(p);
      if (got_b != want_b) {
        if (c->mismatches == 0)
          c->first_bad_off = p;
        c->mismatches++;
      }
    }

    c->received += n;
  }

  QueryPerformanceCounter(&c->t_done);
}

typedef struct {
  int is_pipe;
  SOCKET sock;               /* send socket when !is_pipe */
  HANDLE pipe;               /* send handle when is_pipe */
} probe_sender_t;

typedef struct {
  OVERLAPPED ov;
  int submitted;             /* accepted (sync completion or pending) */
  int sync;                  /* completed synchronously */
  DWORD err;                 /* submission error when !submitted */
  double submit_ms;          /* time spent inside WSASend/WriteFile */
  LARGE_INTEGER t_submit0;
} probe_op_t;

static void probe_op_init(probe_op_t* op) {
  memset(op, 0, sizeof(*op));
  op->ov.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
  ASSERT_NOT_NULL(op->ov.hEvent);
}

static void probe_op_destroy(probe_op_t* op) {
  CloseHandle(op->ov.hEvent);
}

static void probe_submit(probe_sender_t* s,
                         probe_op_t* op,
                         unsigned char* p,
                         unsigned long long len) {
  LARGE_INTEGER t0;
  LARGE_INTEGER t1;
  HANDLE ev;

  ev = op->ov.hEvent;
  ResetEvent(ev);
  memset(&op->ov, 0, sizeof(op->ov));
  op->ov.hEvent = ev;
  op->submitted = 0;
  op->sync = 0;
  op->err = 0;

  if (s->is_pipe) {
    BOOL ok;
    QueryPerformanceCounter(&t0);
    ok = WriteFile(s->pipe, p, (DWORD) len, NULL, &op->ov);
    QueryPerformanceCounter(&t1);
    if (ok) {
      op->submitted = 1;
      op->sync = 1;
    } else {
      DWORD e = GetLastError();
      if (e == ERROR_IO_PENDING)
        op->submitted = 1;
      else
        op->err = e;
    }
  } else {
    WSABUF wb;
    int r;
    wb.buf = (char*) p;
    wb.len = (ULONG) len;
    QueryPerformanceCounter(&t0);
    r = WSASend(s->sock, &wb, 1, NULL, 0, &op->ov, NULL);
    QueryPerformanceCounter(&t1);
    if (r == 0) {
      op->submitted = 1;
      op->sync = 1;
    } else {
      int e = WSAGetLastError();
      if (e == WSA_IO_PENDING)
        op->submitted = 1;
      else
        op->err = (DWORD) e;
    }
  }

  op->t_submit0 = t0;
  op->submit_ms = probe_ms(t0, t1);
}

/* Wait for an accepted submission. 0 on success (bytes filled in); -1 on
 * completion error or timeout (err filled in; (DWORD)-1 means timeout).
 */
static int probe_wait(probe_sender_t* s,
                      probe_op_t* op,
                      DWORD timeout_ms,
                      unsigned long long* bytes,
                      DWORD* err) {
  DWORD w;
  DWORD n = 0;
  BOOL ok;

  *bytes = 0;
  *err = 0;

  w = WaitForSingleObject(op->ov.hEvent, timeout_ms);
  if (w != WAIT_OBJECT_0) {
    *err = (DWORD) -1;
    CancelIoEx(s->is_pipe ? s->pipe : (HANDLE) s->sock, &op->ov);
    WaitForSingleObject(op->ov.hEvent, 10000);
    return -1;
  }

  if (s->is_pipe) {
    ok = GetOverlappedResult(s->pipe, &op->ov, &n, FALSE);
    if (!ok) {
      *err = GetLastError();
      return -1;
    }
  } else {
    DWORD flags = 0;
    ok = WSAGetOverlappedResult(s->sock, &op->ov, &n, FALSE, &flags);
    if (!ok) {
      *err = (DWORD) WSAGetLastError();
      return -1;
    }
  }

  *bytes = n;
  return 0;
}

static int probe_tcp_pair(SOCKET* snd, SOCKET* rcv) {
  SOCKET lst = INVALID_SOCKET;
  SOCKET a = INVALID_SOCKET;
  SOCKET b = INVALID_SOCKET;
  struct sockaddr_in addr;
  int alen;

  lst = socket(AF_INET, SOCK_STREAM, 0);
  if (lst == INVALID_SOCKET)
    return -1;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(lst, (struct sockaddr*) &addr, sizeof(addr)) != 0)
    goto fail;
  if (listen(lst, 1) != 0)
    goto fail;
  alen = sizeof(addr);
  if (getsockname(lst, (struct sockaddr*) &addr, &alen) != 0)
    goto fail;
  a = socket(AF_INET, SOCK_STREAM, 0);  /* overlapped by default */
  if (a == INVALID_SOCKET)
    goto fail;
  if (connect(a, (struct sockaddr*) &addr, sizeof(addr)) != 0)
    goto fail;
  b = accept(lst, NULL, NULL);
  if (b == INVALID_SOCKET)
    goto fail;
  closesocket(lst);
  *snd = a;
  *rcv = b;
  return 0;

fail:
  if (lst != INVALID_SOCKET)
    closesocket(lst);
  if (a != INVALID_SOCKET)
    closesocket(a);
  if (b != INVALID_SOCKET)
    closesocket(b);
  return -1;
}

static int probe_pipe_pair(HANDLE* wr, HANDLE* rd, int idx) {
  char name[128];
  HANDLE srv;
  HANDLE cli;
  OVERLAPPED cov;
  DWORD e;

  snprintf(name,
           sizeof(name),
           "\\\\.\\pipe\\uv-probe-max-write-%lu-%d",
           (unsigned long) GetCurrentProcessId(),
           idx);

  srv = CreateNamedPipeA(name,
                         PIPE_ACCESS_OUTBOUND | FILE_FLAG_OVERLAPPED |
                             FILE_FLAG_FIRST_PIPE_INSTANCE,
                         PIPE_TYPE_BYTE | PIPE_WAIT,
                         1,
                         65536,
                         65536,
                         0,
                         NULL);
  if (srv == INVALID_HANDLE_VALUE)
    return -1;

  /* Reader end is opened synchronous (no FILE_FLAG_OVERLAPPED). */
  cli = CreateFileA(name, GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
  if (cli == INVALID_HANDLE_VALUE) {
    CloseHandle(srv);
    return -1;
  }

  memset(&cov, 0, sizeof(cov));
  cov.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
  if (cov.hEvent == NULL) {
    CloseHandle(srv);
    CloseHandle(cli);
    return -1;
  }
  if (!ConnectNamedPipe(srv, &cov)) {
    e = GetLastError();
    if (e == ERROR_IO_PENDING) {
      if (WaitForSingleObject(cov.hEvent, 10000) != WAIT_OBJECT_0) {
        CloseHandle(cov.hEvent);
        CloseHandle(srv);
        CloseHandle(cli);
        return -1;
      }
    } else if (e != ERROR_PIPE_CONNECTED) {
      CloseHandle(cov.hEvent);
      CloseHandle(srv);
      CloseHandle(cli);
      return -1;
    }
  }
  CloseHandle(cov.hEvent);

  *wr = srv;
  *rd = cli;
  return 0;
}

static void probe_run_ladder(const char* tag,
                             int is_pipe,
                             unsigned char* buf,
                             unsigned long long bufcap,
                             unsigned char* window) {
  unsigned i;
  int consecutive_failures = 0;
  int pipe_idx = 0;

  for (i = 0; i < (unsigned) PROBE_NSIZES; i++) {
    unsigned long long size = probe_ladder[i];
    probe_sender_t snd;
    probe_recv_ctx_t rc;
    probe_op_t op;
    uv_thread_t th;
    SOCKET ss = INVALID_SOCKET;
    SOCKET rs = INVALID_SOCKET;
    HANDLE wp = INVALID_HANDLE_VALUE;
    HANDLE rp = INVALID_HANDLE_VALUE;
    unsigned long long bytes = 0;
    DWORD werr = 0;
    LARGE_INTEGER t_complete;
    double complete_ms = -1.0;
    double drain_ms = -1.0;
    double thr = 0.0;
    int wait_rc = -1;

    if (size > bufcap) {
      printf("#PROBE %s sizeMB=%llu skipped=insufficient_ram\n",
             tag,
             size / PROBE_MB);
      continue;
    }
    if (consecutive_failures >= 2) {
      printf("#PROBE %s sizeMB=%llu skipped=two_consecutive_failures\n",
             tag,
             size / PROBE_MB);
      continue;
    }

    memset(&rc, 0, sizeof(rc));
    memset(&snd, 0, sizeof(snd));
    if (is_pipe) {
      if (probe_pipe_pair(&wp, &rp, pipe_idx++) != 0) {
        printf("#PROBE %s sizeMB=%llu error=pipe_pair gle=%lu\n",
               tag,
               size / PROBE_MB,
               (unsigned long) GetLastError());
        break;
      }
      snd.is_pipe = 1;
      snd.pipe = wp;
      rc.is_pipe = 1;
      rc.pipe = rp;
    } else {
      if (probe_tcp_pair(&ss, &rs) != 0) {
        printf("#PROBE %s sizeMB=%llu error=tcp_pair wsa=%d\n",
               tag,
               size / PROBE_MB,
               WSAGetLastError());
        break;
      }
      snd.is_pipe = 0;
      snd.sock = ss;
      rc.is_pipe = 0;
      rc.sock = rs;
    }
    rc.window = window;
    rc.window_size = (size_t) PROBE_RECV_WINDOW;

    probe_fill(buf, 0, size);
    probe_op_init(&op);
    ASSERT_OK(uv_thread_create(&th, probe_recv_thread, &rc));

    probe_submit(&snd, &op, buf, size);
    if (op.submitted) {
      wait_rc = probe_wait(&snd, &op, PROBE_WAIT_MS, &bytes, &werr);
      QueryPerformanceCounter(&t_complete);
      complete_ms = probe_ms(op.t_submit0, t_complete);
    }

    if (op.submitted && wait_rc == 0 && bytes == size)
      consecutive_failures = 0;
    else
      consecutive_failures++;

    /* EOF the reader, then collect it. */
    if (is_pipe)
      CloseHandle(wp);
    else
      shutdown(ss, SD_SEND);
    ASSERT_OK(uv_thread_join(&th));
    if (!is_pipe) {
      closesocket(ss);
      closesocket(rs);
    } else {
      CloseHandle(rp);
    }

    if (op.submitted) {
      drain_ms = probe_ms(op.t_submit0, rc.t_done);
      if (drain_ms > 0.0)
        thr = ((double) rc.received / (double) PROBE_MB) / (drain_ms / 1000.0);
    }

    printf("#PROBE %s sizeMB=%llu submit=%s err=%lu submit_ms=%.3f "
           "complete_ms=%.1f completed_bytes=%llu drain_ms=%.1f "
           "recvMB=%llu thr_MBps=%.0f marker_mismatches=%llu "
           "first_bad_off=%llu\n",
           tag,
           size / PROBE_MB,
           op.submitted ? (op.sync ? "sync" : "pending") : "REJECTED",
           (unsigned long) (op.submitted ? werr : op.err),
           op.submit_ms,
           complete_ms,
           bytes,
           drain_ms,
           rc.received / PROBE_MB,
           thr,
           rc.mismatches,
           rc.first_bad_off);

    probe_op_destroy(&op);
  }
}

/* Compare one outstanding overlapped WSASend vs two outstanding on a single
 * TCP connection, at a fixed chunk size, and check that data from two
 * outstanding sends lands on the stream in submission order.
 */
static void probe_run_multi(unsigned char* buf,
                            unsigned long long bufcap,
                            unsigned char* window) {
  unsigned long long chunk = 512 * PROBE_MB;
  int cfg;

  if (bufcap < 2 * chunk)
    chunk = 256 * PROBE_MB;
  if (bufcap < 2 * chunk) {
    printf("#PROBE multi skipped=insufficient_ram\n");
    return;
  }

  for (cfg = 1; cfg <= 2; cfg++) {
    SOCKET ss = INVALID_SOCKET;
    SOCKET rs = INVALID_SOCKET;
    probe_recv_ctx_t rc;
    probe_sender_t snd;
    probe_op_t ops[2];
    uv_thread_t th;
    unsigned long long total = chunk * PROBE_MULTI_CHUNKS;
    LARGE_INTEGER t0;
    double wall_ms;
    double thr = 0.0;
    int aborted = 0;
    int comp_seq[PROBE_MULTI_CHUNKS];
    int comp_count = 0;
    int completions_in_order = 1;
    int k;

    if (probe_tcp_pair(&ss, &rs) != 0) {
      printf("#PROBE multi cfg=K%d error=tcp_pair wsa=%d\n",
             cfg,
             WSAGetLastError());
      return;
    }
    memset(&rc, 0, sizeof(rc));
    rc.sock = rs;
    rc.window = window;
    rc.window_size = (size_t) PROBE_RECV_WINDOW;
    memset(&snd, 0, sizeof(snd));
    snd.sock = ss;
    probe_op_init(&ops[0]);
    probe_op_init(&ops[1]);
    ASSERT_OK(uv_thread_create(&th, probe_recv_thread, &rc));

    QueryPerformanceCounter(&t0);

    if (cfg == 1) {
      /* One op outstanding: submit, wait, submit next. */
      int i;
      for (i = 0; i < PROBE_MULTI_CHUNKS && !aborted; i++) {
        unsigned long long bytes;
        DWORD werr;
        probe_fill(buf, (unsigned long long) i * chunk, chunk);
        probe_submit(&snd, &ops[0], buf, chunk);
        if (!ops[0].submitted ||
            probe_wait(&snd, &ops[0], PROBE_WAIT_MS, &bytes, &werr) != 0 ||
            bytes != chunk) {
          aborted = 1;
          break;
        }
        comp_seq[comp_count++] = i;
      }
    } else {
      /* Two ops outstanding, double-buffered from two buffer regions. */
      int next = 0;
      int inflight = 0;
      int inflight_flag[2] = {0, 0};
      int region_chunk[2] = {-1, -1};
      HANDLE evs[2];

      evs[0] = ops[0].ov.hEvent;
      evs[1] = ops[1].ov.hEvent;

      for (next = 0; next < 2 && !aborted; next++) {
        probe_fill(buf + (unsigned long long) next * chunk,
                   (unsigned long long) next * chunk,
                   chunk);
        probe_submit(&snd,
                     &ops[next],
                     buf + (unsigned long long) next * chunk,
                     chunk);
        region_chunk[next] = next;
        if (!ops[next].submitted)
          aborted = 1;
        else {
          inflight_flag[next] = 1;
          inflight++;
        }
      }

      while (inflight > 0 && !aborted) {
        DWORD w = WaitForMultipleObjects(2, evs, FALSE, PROBE_WAIT_MS);
        int idx;
        unsigned long long bytes;
        DWORD werr;

        if (w == WAIT_OBJECT_0)
          idx = 0;
        else if (w == WAIT_OBJECT_0 + 1)
          idx = 1;
        else {
          aborted = 1;
          break;
        }
        if (!inflight_flag[idx]) {
          ResetEvent(evs[idx]);  /* stale signal from a finished op */
          continue;
        }
        if (probe_wait(&snd, &ops[idx], 0, &bytes, &werr) != 0 ||
            bytes != chunk) {
          aborted = 1;
          break;
        }
        ResetEvent(evs[idx]);
        inflight_flag[idx] = 0;
        inflight--;
        comp_seq[comp_count++] = region_chunk[idx];

        if (next < PROBE_MULTI_CHUNKS) {
          probe_fill(buf + (unsigned long long) idx * chunk,
                     (unsigned long long) next * chunk,
                     chunk);
          probe_submit(&snd,
                       &ops[idx],
                       buf + (unsigned long long) idx * chunk,
                       chunk);
          region_chunk[idx] = next;
          if (!ops[idx].submitted) {
            aborted = 1;
            break;
          }
          inflight_flag[idx] = 1;
          inflight++;
          next++;
        }
      }
    }

    shutdown(ss, SD_SEND);
    ASSERT_OK(uv_thread_join(&th));
    closesocket(ss);
    closesocket(rs);

    for (k = 1; k < comp_count; k++)
      if (comp_seq[k] < comp_seq[k - 1])
        completions_in_order = 0;

    wall_ms = probe_ms(t0, rc.t_done);
    if (wall_ms > 0.0)
      thr = ((double) rc.received / (double) PROBE_MB) / (wall_ms / 1000.0);

    printf("#PROBE multi cfg=K%d chunkMB=%llu chunks=%d totalMB=%llu "
           "wall_ms=%.1f thr_MBps=%.0f recvMB=%llu marker_mismatches=%llu "
           "first_bad_off=%llu completions_in_submission_order=%s "
           "aborted=%d\n",
           cfg,
           chunk / PROBE_MB,
           PROBE_MULTI_CHUNKS,
           total / PROBE_MB,
           wall_ms,
           thr,
           rc.received / PROBE_MB,
           rc.mismatches,
           rc.first_bad_off,
           completions_in_order ? "yes" : "NO",
           aborted);

    probe_op_destroy(&ops[0]);
    probe_op_destroy(&ops[1]);
  }
}

TEST_IMPL(probe_max_write) {
  MEMORYSTATUSEX msx;
  WSADATA wsad;
  unsigned char* buf = NULL;
  unsigned char* window = NULL;
  unsigned long long bufcap = 0;
  unsigned long long slack;
  int i;
  DWORD ver_major = 0;
  DWORD ver_minor = 0;
  DWORD ver_build = 0;

  setvbuf(stdout, NULL, _IONBF, 0);

  ASSERT(QueryPerformanceFrequency(&probe_freq));

  memset(&msx, 0, sizeof(msx));
  msx.dwLength = sizeof(msx);
  ASSERT(GlobalMemoryStatusEx(&msx));

  {
    typedef LONG(WINAPI * rtlgetversion_t)(RTL_OSVERSIONINFOW*);
    rtlgetversion_t rtl_get_version;
    HMODULE ntdll = GetModuleHandleA("ntdll.dll");
    if (ntdll != NULL) {
      rtl_get_version = (rtlgetversion_t)(void (*)(void))
          GetProcAddress(ntdll, "RtlGetVersion");
      if (rtl_get_version != NULL) {
        RTL_OSVERSIONINFOW ver;
        memset(&ver, 0, sizeof(ver));
        ver.dwOSVersionInfoSize = sizeof(ver);
        if (rtl_get_version(&ver) == 0) {
          ver_major = ver.dwMajorVersion;
          ver_minor = ver.dwMinorVersion;
          ver_build = ver.dwBuildNumber;
        }
      }
    }
  }

  printf("#PROBE meta win=%lu.%lu.%lu totalMB=%llu availMB=%llu\n",
         (unsigned long) ver_major,
         (unsigned long) ver_minor,
         (unsigned long) ver_build,
         msx.ullTotalPhys / PROBE_MB,
         msx.ullAvailPhys / PROBE_MB);

  if (msx.ullTotalPhys < 8ULL * 1024 * PROBE_MB)
    RETURN_SKIP("probe_max_write requires at least 8 GB of RAM.");

  ASSERT_OK(WSAStartup(MAKEWORD(2, 2), &wsad));

  /* Allocate the largest ladder rung that leaves ample slack, stepping
   * down on VirtualAlloc failure.
   */
  slack = 1536 * PROBE_MB;
  for (i = (int) PROBE_NSIZES - 1; i >= 0; i--) {
    if (probe_ladder[i] + slack > msx.ullAvailPhys)
      continue;
    buf = (unsigned char*) VirtualAlloc(NULL,
                                        (SIZE_T) probe_ladder[i],
                                        MEM_RESERVE | MEM_COMMIT,
                                        PAGE_READWRITE);
    if (buf != NULL) {
      bufcap = probe_ladder[i];
      break;
    }
  }
  if (buf == NULL) {
    WSACleanup();
    RETURN_SKIP("probe_max_write could not allocate a probe buffer.");
  }

  window = (unsigned char*) VirtualAlloc(NULL,
                                         (SIZE_T) PROBE_RECV_WINDOW,
                                         MEM_RESERVE | MEM_COMMIT,
                                         PAGE_READWRITE);
  ASSERT_NOT_NULL(window);

  /* Fault every page in ahead of time so submission timing measures the
   * kernel's probe-and-lock, not first-touch page zeroing.
   */
  memset(buf, 0xAA, (size_t) bufcap);

  printf("#PROBE meta bufMB=%llu recv_windowMB=%llu\n",
         bufcap / PROBE_MB,
         PROBE_RECV_WINDOW / PROBE_MB);

  probe_run_ladder("tcp", 0, buf, bufcap, window);
  probe_run_ladder("pipe", 1, buf, bufcap, window);
  probe_run_multi(buf, bufcap, window);

  VirtualFree(buf, 0, MEM_RELEASE);
  VirtualFree(window, 0, MEM_RELEASE);
  WSACleanup();
  return 0;
}

#endif /* 64-bit Windows */
