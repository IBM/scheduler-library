TARGET=libscheduler.a
TTARGET=libtasks.a

EXAMPLES=mini-era demo-me demo-tall

#$(info $$TARGET is [${TARGET}])
#$(info $$EXAMPLES is [${EXAMPLES}])


all: $(STARGET) $(TTARGET) $(EXAMPLES)

libscheduler.a:
	(cd sched_library; $(MAKE))

libtasks.a:
	(cd task_library; $(MAKE))

mini-era: $(STARGET) $(TTARGET)
	(cd examples ; make mini-era-build)

demo-me: $(STARGET) $(TTARGET)
	(cd examples ; make demo-me-build)

demo-tall: $(STARGET) $(TTARGET)
	(cd examples ; make demo-tall-build)

clean:
	(cd sched_library; $(MAKE) clean)
	(cd task_library; $(MAKE) clean)
	(cd examples; $(MAKE) clean)

clobber: clean
	(cd sched_library; $(MAKE) clobber)
	(cd task_library; $(MAKE) clobber)
	(cd examples; $(MAKE) clobber)

.PHONY: all clean clobber

