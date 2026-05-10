# Findings

- Pending.
- The Max Carrier bench notes now pin the Murata L072 bootloader-entry pins on the H747 side as PA_11 for BOOT0 and PF_4 for NRST.
- The H747-side PA_11 high test was not enough to recover user-mode ingress, so the remaining gate is not BOOT0 alone.
- The H747 remains the controller-side owner of the carrier UART routing and should be treated as the board-level place to look for any mux, enable, or flow-control gating.
