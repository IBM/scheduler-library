STARGET=libscheduler.a
TTARGET=libtasks.a

EXAMPLES=mini-era #demo-me

#$(info $$TARGET is [${TARGET}])
#$(info $$EXAMPLES is [${EXAMPLES}])


all: $(STARGET) $(TTARGET) $(EXAMPLES)

libscheduler.a:
	(cd sched_library; $(MAKE))

libtasks.a:
	(cd task_library; $(MAKE))

hpvm-cpu: libscheduler.a libtasks.a 
	(cd examples/mini-era/ ; make hpvm-cpu)
	(cd examples/demo-me/ ; make hpvm-cpu)
	#(cd examples/mini-era/ ; make CONFIG_VERBOSE=y hpvm-cpu)

hpvm-epochs: libscheduler.a libtasks.a
	(cd examples/mini-era/ ; make hpvm-epochs)
	(cd examples/demo-me/ ; make hpvm-epochs)
	#(cd examples/mini-era/ ; make CONFIG_VERBOSE=y hpvm-epochs)

mini-era: $(TARGET)
	(cd examples ; make mini-era-build)

demo-me: $(TARGET)
	(cd examples ; make demo-me-build)

clean:
	(cd sched_library; $(MAKE) clean)
	(cd task_library; $(MAKE) clean)
	(cd examples; $(MAKE) clean)

clobber: clean
	(cd sched_library; $(MAKE) clobber)
	(cd task_library; $(MAKE) clobber)
	(cd examples; $(MAKE) clobber)

.PHONY: all clean clobber

