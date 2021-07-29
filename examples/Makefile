
all: mini-era-build demo-me-build

mini-era-build:
	(cd mini-era ; $(MAKE))

demo-me-build:
	(cd demo-me ; $(MAKE))

clean:
	(cd mini-era; $(MAKE) clean)
	(cd demo-me; $(MAKE) clean)

clobber: clean
	(cd mini-era; $(MAKE) clobber)
	(cd demo-me; $(MAKE) clobber)

.PHONY: all clean clobber

