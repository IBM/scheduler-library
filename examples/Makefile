
all: mini-era-build demo-me-build

mini-era-build:
	(cd mini-era ; $(MAKE))

demo-me-build:
	(cd demo-me ; $(MAKE))

demo-tall-build:
	(cd demo-tall ; $(MAKE))

clean:
	(cd mini-era; $(MAKE) clean)
	(cd demo-me; $(MAKE) clean)
	(cd demo-tall; $(MAKE) clean)

clobber: clean
	(cd mini-era; $(MAKE) clobber)
	(cd demo-me; $(MAKE) clobber)
	(cd demo-tall; $(MAKE) clobber)

.PHONY: all clean clobber

