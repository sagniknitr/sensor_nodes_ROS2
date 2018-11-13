* QNX requires memory pages to be marked as executable.
Otherwise, the OS raises an exception when executing code in that page.

*  0 is never a valid thread id on Qnx since tids and pids share a
 name space and pid 0 is reserved (see man 2 kill).
