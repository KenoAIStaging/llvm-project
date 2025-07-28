; RUN: opt -passes=irce -irce-skip-profitability-checks \
; RUN:   -verify-analysis-invalidation -S < %s | FileCheck %s
; RUN: opt -passes=irce -irce-skip-profitability-checks \
; RUN:   -irce-allow-narrow-latch=false -verify-analysis-invalidation -S \
; RUN:   < %s | FileCheck %s --check-prefix=NARROW

; Parsing the loop structure may need to materialize SCEVs in the preheader.
; If a later legality check prevents cloning, those speculative instructions
; must be removed before IRCE reports that it did not change the function.

declare void @cannot_duplicate() noduplicate
declare void @convergent_call() convergent
declare void @sideeffect()

define void @no_materialization(ptr %n.ptr) {
; CHECK-LABEL: define void @no_materialization(
entry:
; CHECK: entry:
; CHECK-NEXT: %n = load i32, ptr %n.ptr, align 4, !range !0
; CHECK-NEXT: br label %loop
  %n = load i32, ptr %n.ptr, !range !0
  br label %loop

loop:
  %idx = phi i32 [ 0, %entry ], [ %idx.next, %in.bounds ]
  %idx.next = add i32 %idx, 1
  %bound = add nsw i32 %n, 1
  %in.range = icmp slt i32 %idx, 50
  call void @cannot_duplicate()
  br i1 %in.range, label %in.bounds, label %out.of.bounds

in.bounds:
; CHECK: %bound = add nsw i32 %n, 1
  %next = icmp slt i32 %idx.next, %bound
  br i1 %next, label %loop, label %exit

out.of.bounds:
  ret void

exit:
  ret void
}

!0 = !{i32 1, i32 1000}

; A loop that does not need pre- or post-loop clones may still be constrained
; when its body is unsafe to clone.

define void @no_cloning_needed(i32 range(i32 100, 150) %len,
                               i32 range(i32 1, 50) %n) {
; CHECK-LABEL: define void @no_cloning_needed(
entry:
  br label %loop

loop:
  %idx = phi i32 [ 0, %entry ], [ %idx.next, %in.bounds ]
  %in.range = icmp slt i32 %idx, %len
  call void @convergent_call()
; CHECK: br i1 true, label %in.bounds, label %out.of.bounds
  br i1 %in.range, label %in.bounds, label %out.of.bounds

in.bounds:
  %idx.next = add nuw nsw i32 %idx, 1
  %next = icmp slt i32 %idx.next, %n
  br i1 %next, label %loop, label %exit

out.of.bounds:
  ret void

exit:
  ret void
}

; The pre-loop limit can be expanded, but the post-loop limit contains a
; conditionally executed division which cannot safely be expanded in the
; preheader. Make sure the partially expanded pre-loop limit is removed and
; the name of the reused induction start is left unchanged.

define void @cleanup_after_partial_expansion(ptr %num.ptr, ptr %denom.ptr,
                                             i64 range(i64 -100, -1) %offset,
                                             i64 range(i64 0, 10) %start,
                                             i1 %maybe.exit) {
; CHECK-LABEL: define void @cleanup_after_partial_expansion(
; CHECK-SAME: i64 range(i64 0, 10) %start,
entry:
; CHECK: entry:
; CHECK-NEXT: %num = load i64, ptr %num.ptr, align 8, !range !1
; CHECK-NEXT: %denom = load i64, ptr %denom.ptr, align 8, !range !1
; CHECK-NEXT: br label %loop
  %num = load i64, ptr %num.ptr, align 8, !range !1
  %denom = load i64, ptr %denom.ptr, align 8, !range !1
  br label %loop

exit:
  ret void

loop:
  %iv = phi i64 [ %start, %entry ], [ %iv.next, %guarded ]
  %checked = phi i64 [ %offset, %entry ], [ %checked.next, %guarded ]
  %iv.next = add nuw nsw i64 %iv, 1
  br i1 %maybe.exit, label %range.check, label %exit

range.check:
  %div.result = udiv i64 %num, %denom
  %rc = icmp slt i64 %checked, %div.result
  br i1 %rc, label %guarded, label %exit

guarded:
  %checked.next = add nsw i64 %checked, 1
  call void @sideeffect()
  %loop.cond = icmp slt i64 %iv.next, 1000
  br i1 %loop.cond, label %loop, label %exit
}

; IRCE canonicalizes range-check branches before calculating the constrained
; subranges. If mismatched types make that calculation fail, the branch
; inversion must still be reported as a change.

define void @inversion_before_subrange_failure() {
; NARROW-LABEL: define void @inversion_before_subrange_failure(
; NARROW: %range.check.failed = icmp slt i64 %iv, 100
; NARROW-NEXT: br i1 %range.check.failed, label %backedge, label %check.failed
entry:
  br label %loop

loop:
  %iv = phi i64 [ 0, %entry ], [ %iv.next, %backedge ]
  %range.check.failed = icmp sge i64 %iv, 100
  br i1 %range.check.failed, label %check.failed, label %backedge

backedge:
  %iv.next = add i64 %iv, 1
  %narrow.iv = trunc i64 %iv.next to i32
  %latch.cond = icmp slt i32 %narrow.iv, 100
  br i1 %latch.cond, label %loop, label %exit

exit:
  ret void

check.failed:
  ret void
}

!1 = !{i64 0, i64 100}
