---
ortho-router: patch
---

A connector between nodes closer than twice the margin is routed instead
of falling back to the elbow: a connector with no route is tried again
with half the margin, up to three times. Spreading wires that share a
channel stays within the margin their routes were found with.
