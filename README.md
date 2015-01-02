README
======

What is this?
-------------

This is the linux 'radeon', hacked to permit fine-grained reverse-engineering
of the ATOMBIOS on which this driver so carelessly depends. It contains a number
of non-upstreamable hacks, from IO tracing of atom table execution, to
re-implementations of atom tables which could be upstreamed in the future.

It is based on the 3.17 kernel release, but could be rebased on newer versions
in the future.

Quick and dirty HOWTO
---------------------

Find an atombios that you'd like to RE. Decide if you want iotracing only on
certain parameters. Enable iotracing before calling the table, then disable it
once the table returns. iotracing is controlled by the global 'atombios_iotrace'
variable.

	extern int atombios_iotrace;
	
	...
	
	if (condition)
		atombios_iotrace = 1;
	atom_execute_table(rdev->mode_info.atom_context, index, (uint32_t *)&args);
	atombios_iotrace = 0;

Build and install the hacked module (see below). Unload old radeon module, then
> modprobe radeon

This may require a system reboot, as unbinding the GPU device from the driver
will severely confuse the system and hardware.


Build
-----

One way is to create a makefile which points to the radeon sources. Then,
commands like 'make modules' and 'make install' will work as expected.

	KDIR  := /lib/modules/$(shell uname -r)/build
	SRC   := /path/to/radeon
	
	all: modules
	
	modules:
		$(MAKE) -C $(KDIR) -r M=$(SRC) modules
	
	clean:
		$(MAKE) -C $(KDIR) -r M=$(SRC) clean
	
	install:
	        $(MAKE) -C $(KDIR) -r M=$(SRC) modules_install

Another way is to cd to the radeon sources and use the following commands:

	make -C /lib/modules/`uname -r`/build M=`pwd` clean
	make -C /lib/modules/`uname -r`/build M=`pwd` modules
	make -C /lib/modules/`uname -r`/build M=`pwd` modules_install

External resources
------------------

* http://github.com/alterapraxisptyltd/openatom
* http://cgit.freedesktop.org/~mhopf/AtomDis/