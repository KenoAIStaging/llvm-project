; RUN: opt < %s -passes=sink -S | FileCheck %s
; XFAIL: *
;
; The Sink pass currently sinks read-only calls past atomic stores and RMW
; operations with release (or stronger) ordering, which is a miscompile:
; moving a program-order-earlier read below a release operation breaks the
; publication guarantee. Given a second thread executing
;
;   if (atomic_load_explicit(&flag, memory_order_acquire) == 1)
;     data = 42;
;
; the original @no_sink_call_past_release_store is race-free: the call's read
; of @data happens-before the release store to @flag, which synchronizes-with
; the acquire load, which happens-before the second thread's store to @data.
; After sinking the call below the release store, the read of @data is no
; longer ordered before the second thread's store: the transformed program
; has a data race and can observe the value 42, which the original program
; could not.
;
; Root cause: AAResults::getModRefInfo(const Instruction *, const CallBase *)
; never consults the atomic ordering of the non-call instruction. It only
; asks whether the call accesses the store's own location and returns
; NoModRef otherwise, unlike the per-instruction getModRefInfo overloads,
; which conservatively report the synchronization effects (getSyncEffects) of
; atomic operations stronger than monotonic. Note that a purely conservative
; fix in that overload would pessimize legal *hoisting* of read-only calls
; across release stores (e.g. by LICM), which goes through the same query;
; see the discussion of direction-aware handling in the fix for the LICM
; load-hoisting case.
;
; The CHECK lines below describe the intended (correct) behavior: the call
; must stay above the release/seq_cst operation.

@flag = global i32 0
@data = global i32 0

declare i32 @read_data(ptr) nounwind willreturn memory(argmem: read)

define i32 @no_sink_call_past_release_store(i1 %c) {
; CHECK-LABEL: @no_sink_call_past_release_store(
; CHECK:         [[V:%.*]] = call i32 @read_data(ptr @data)
; CHECK-NEXT:    store atomic i32 1, ptr @flag release
entry:
  %v = call i32 @read_data(ptr @data)
  store atomic i32 1, ptr @flag release, align 4
  br i1 %c, label %use, label %skip

use:
  ret i32 %v

skip:
  ret i32 0
}

define i32 @no_sink_call_past_seq_cst_store(i1 %c) {
; CHECK-LABEL: @no_sink_call_past_seq_cst_store(
; CHECK:         [[V:%.*]] = call i32 @read_data(ptr @data)
; CHECK-NEXT:    store atomic i32 1, ptr @flag seq_cst
entry:
  %v = call i32 @read_data(ptr @data)
  store atomic i32 1, ptr @flag seq_cst, align 4
  br i1 %c, label %use, label %skip

use:
  ret i32 %v

skip:
  ret i32 0
}

define i32 @no_sink_call_past_release_rmw(i1 %c) {
; CHECK-LABEL: @no_sink_call_past_release_rmw(
; CHECK:         [[V:%.*]] = call i32 @read_data(ptr @data)
; CHECK-NEXT:    {{%.*}} = atomicrmw add ptr @flag, i32 1 release
entry:
  %v = call i32 @read_data(ptr @data)
  %old = atomicrmw add ptr @flag, i32 1 release, align 4
  br i1 %c, label %use, label %skip

use:
  ret i32 %v

skip:
  ret i32 0
}
