TARGET=libscheduler.a

EXAMPLES=mini-era

#$(info $$TARGET is [${TARGET}])
#$(info $$EXAMPLES is [${EXAMPLES}])


all: $(TARGET) $(EXAMPLES)

libscheduler.a:
	(cd library; $(MAKE))

mini-era: $(TARGET)
	(cd examples ; make mini-era-build)

clean:
	(cd library; $(MAKE) clean)
	(cd examples; $(MAKE) clean)

clobber: clean
	(cd library; $(MAKE) clobber)
	(cd examples; $(MAKE) clobber)

.PHONY: all clean clobber

