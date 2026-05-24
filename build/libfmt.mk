# Set this option to use libfmt in header-only mode; this is necessary
# for example if you want to use libc++, but the libfmt in your
# distribution was built with libstdc++.
ifeq ($(CLANG),y)
FMT_HEADER_ONLY ?= y
else
FMT_HEADER_ONLY ?= n
endif

$(eval $(call pkg-config-library,LIBFMT,fmt '>=' 7))

FMT_SOURCES = \
	$(SRC)/lib/fmt/RuntimeError.cxx \
	$(SRC)/lib/fmt/SystemError.cxx

FMT_CPPFLAGS = $(LIBFMT_CPPFLAGS)

ifeq ($(FMT_HEADER_ONLY),y)
FMT_CPPFLAGS += -DFMT_HEADER_ONLY
else
FMT_LDLIBS += $(LIBFMT_LDLIBS)
endif

$(eval $(call link-library,fmt,FMT))
