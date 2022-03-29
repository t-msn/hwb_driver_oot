Hardware barrier driver for A64FX.

This is rather an experimental implementation and under development (built and tested on 5.17)

This module creates following sysfs:
```
/sys/class/misc/hard_barrier/
  |- group0
    |- available_cpus // 0-11 for A64Fx
    |- barrier0
      |- masks
      |- task
      |- user
    |- barrier1
    ...
  |- group1
    |- available_cpus // 11-23 for A64Fx
    |- barrier0
      |- masks
      |- task
      |- user
  ...
```

Usage:
 1. A process who wants to use barrier first writes cpu masks (which
    represents cores participating barrier) to 'barrierN/masks'
    Which core can be used in that  barrier is shown in 'groupN/available_cpus'.
    If barrier is already used, write returns error.
    (Read to 'barrierN/task' shows which pid uses the barrier.)
 2. A thread who joins the barrier must has the same PID of (1).
    Read to 'barrierN/user' will return barrier window number to be
    used in userland. At this point a thread must be pinned at one CPU.
 3. (Synchronization is performed in userland)
 4. Write "" to 'barrierN/masks' will release barrier resource

Note that write to sysfs is allowed only for root by default.
Change sysfs's permission appropriately for non-root user.


Build:
```
$ make LINUX=/path/to/linux
