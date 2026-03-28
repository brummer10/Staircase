

	# check if user is root
	user = $(shell whoami)
	ifeq ($(user),root)
	INSTALL_DIR = /usr/lib/lv2
	else 
	INSTALL_DIR = ~/.lv2
	endif




	NAME = Staircase
	space := $(subst ,, )
	EXEC_NAME := $(subst $(space),_,$(NAME))
	BUNDLE = $(EXEC_NAME).lv2
	RESOURCES_DIR :=../resources/
	LIB_DIR := ../libxputty/libxputty/
	HEADER_DIR := $(LIB_DIR)include/
	UI_LIB:= $(LIB_DIR)libxputty.$(STATIC_LIB_EXT)
	STRIP ?= strip

	RESOURCES := $(wildcard $(RESOURCES_DIR)*.png)
	RESOURCES_OBJ := $(notdir $(patsubst %.png,%.o,$(RESOURCES)))
	RESOURCES_LIB := $(notdir $(patsubst %.png,%.$(STATIC_LIB_EXT),$(RESOURCES)))
	RESOURCE_EXTLD := $(notdir $(patsubst %.png,%_png,$(RESOURCES)))
	RESOURCEHEADER := xresources.h
ifeq ($(TARGET), Linux)
	GUI_LDFLAGS += -I$(HEADER_DIR) -Wl,-Bstatic -L. $(UI_LIB) \
	-Wl,-Bdynamic `pkg-config --cflags --libs cairo x11 fftw3f` 
	LDFLAGS += -fvisibility=hidden -shared -lm -fPIC -pthread -lpthread\
	-Wl,-z,noexecstack -Wl,--no-undefined -Wl,--gc-sections
	CXXFLAGS += -fPIC -DPIC -O2 -Wall -funroll-loops \
	-fomit-frame-pointer -fstrength-reduce -fstack-protector -fvisibility=hidden \
	-fdata-sections -Wl,--gc-sections -Wl,--exclude-libs,ALL
	SLP_LDFLAGS += -fvisibility=hidden -Wl,-Bstatic `pkg-config --cflags --libs xputty` \
	-Wl,-Bdynamic `pkg-config --cflags --libs cairo x11 lilv-0` \
	-shared -lm -fPIC -Wl,-z,noexecstack -Wl,--no-undefined -Wl,--gc-sections
	CFLAGS := -O2 -D_FORTIFY_SOURCE=2 -Wall -fstack-protector -fvisibility=hidden \
	-fdata-sections -Wl,--gc-sections -Wl,-z,relro,-z,now -Wl,--exclude-libs,ALL
else ifeq ($(TARGET), Windows)
	CXXFLAGS += -D_FORTIFY_SOURCE=2 -I. -I./dsp -I./plugin -fPIC -DPIC -O2 -Wall -funroll-loops \
	-fstack-protector -ffast-math -fomit-frame-pointer -fstrength-reduce \
	-fdata-sections -Wl,--gc-sections -Wl,--exclude-libs,ALL $(SSE_CFLAGS)
	LDFLAGS += -I. -shared -lm $(PAWPAW_LFLAGS)
	GUI_LDFLAGS += -I$(HEADER_DIR) -static-libgcc -static-libstdc++ \
	`$(PKGCONFIG) $(PKGCONFIG_FLAGS) --cflags --libs cairo ` \
	-L. $(LIB_DIR)libxputty.$(STATIC_LIB_EXT) -shared  -lm $(PAWPAW_LFLAGS)
	TTLUPDATE = sed -i '/lv2:binary/ s/\.so/\.dll/ ' ../bin/$(BUNDLE)/manifest.ttl
	TTLUPDATEGUI = sed -i '/a guiext:X11UI/ s/X11UI/WindowsUI/ ; /guiext:binary/ s/\.so/\.dll/ ' ../bin/$(BUNDLE)/$(NAME).ttl
endif

	FAUSTFLAGS :=  \
	 \
	

.PHONY : all install uninstall

.NOTPARALLEL:

all: $(RESOURCEHEADER) $(EXEC_NAME)

	$(QUIET)mkdir -p ../bin/$(BUNDLE)
	$(QUIET)cp ./*.ttl ../bin/$(BUNDLE)
	$(QUIET)$(TTLUPDATE)
	$(QUIET)$(TTLUPDATEGUI)
	$(QUIET)cp ./*.$(LIB_EXT) ../bin/$(BUNDLE)
	$(QUIET)if [ -f ../bin/$(BUNDLE)/$(EXEC_NAME).$(LIB_EXT) ]; then \
		$(B_ECHO) "build finish, . . . $(reset)"; \
	else \
		$(R_ECHO) "Sorry, build fail"; \
	fi
	@$(B_ECHO) "=================== DONE =======================$(reset)"

$(RESOURCEHEADER): $(RESOURCES_OBJ)
	$(QUIET)rm -f $(RESOURCEHEADER)
	$(QUIET)for f in $(RESOURCE_EXTLD); do \
		echo 'EXTLD('$${f}')' >> $(RESOURCEHEADER) ; \
	done

ifdef XXDI
$(RESOURCES_OBJ): $(RESOURCES)
	@#use this line to regenerate the *.c files from used images
	@#cd $(RESOURCES_DIR) && xxd -i $(patsubst %.o,%.png,$@) > $(patsubst %.o,%.c,$@)
	$(QUIET)$(CC) -c $(RESOURCES_DIR)$(patsubst %.o,%.c,$@) -o $@
	$(QUIET)$(AR) rcs $(patsubst %.o,%.$(STATIC_LIB_EXT),$@) $@
else ifdef USE_LDD
$(RESOURCES_OBJ): $(RESOURCES)
	$(QUIET)cd $(RESOURCES_DIR) && $(LD) -r -b binary -m $(LDEMULATION) $(patsubst %.o,%.png,$@) -o ../$@
	$(QUIET)$(AR) rcs $(patsubst %.o,%.$(STATIC_LIB_EXT),$@) $@
else
$(RESOURCES_OBJ): $(RESOURCES)
	$(QUIET)cd $(RESOURCES_DIR) && $(LD) -r -b binary --strip-all $(patsubst %.o,%.png,$@) -o ../$@
	$(QUIET)$(AR) rcs $(patsubst %.o,%.$(STATIC_LIB_EXT),$@) $@
endif

$(EXEC_NAME):$(RESOURCES_OBJ)
	@$(B_ECHO) "Compiling $(NAME).lv2 $(reset)"
	@# use this line when you include libxputty as submodule
	$(QUIET)$(CC)   $(CFLAGS) '$(NAME).c' -L. $(RESOURCES_LIB) $(UI_LIB) -o '$(EXEC_NAME)_ui.$(LIB_EXT)' $(LDFLAGS) $(GUI_LDFLAGS) -I./
	$(QUIET)$(CXX) $(CXXFLAGS) $(FAUSTFLAGS) $(EXEC_NAME).cpp $(LDFLAGS) -o $(EXEC_NAME).$(LIB_EXT)
	@#$(CC)   $(CFLAGS) '$(NAME).c' -L. $(RESOURCES_LIB) -o '$(EXEC_NAME)_ui.$(LIB_EXT)' $(SLP_LDFLAGS) -I./
	$(QUIET)$(STRIP) -s -x -X -R .comment -R .note.ABI-tag $(EXEC_NAME).$(LIB_EXT)
	$(QUIET)$(STRIP) -s -x -X -R .comment -R .note.ABI-tag $(EXEC_NAME)_ui.$(LIB_EXT)

install :
ifeq ($(TARGET), Linux)
ifneq ("$(wildcard ../bin/$(BUNDLE))","")
	@$(B_ECHO) "Install $(NAME).lv2 $(reset)"
	$(QUIET)mkdir -p $(DESTDIR)$(INSTALL_DIR)/$(BUNDLE)
	$(QUIET)cp -r ../bin/$(BUNDLE)/* $(DESTDIR)$(INSTALL_DIR)/$(BUNDLE)
	$(QUIET)echo ". ., done"
else
	$(QUIET)echo ". ., you must build first"
endif

else
	$(QUIET)$(R_ECHO) "Install is not implemented for windows, please copy the folder $(NAME).lv2 to Program Files/Common Files/LV2$(reset)"
endif

uninstall :
ifeq ($(TARGET), Linux)
	@$(B_ECHO) "Uninstall $(NAME).lv2 $(reset)"
	$(QUIET)rm -rf $(INSTALL_DIR)/$(BUNDLE)
	$(QUIET)echo ". ., done"

else
	$(QUIET)$(R_ECHO) "Uninstall is not implemented for windows, please delete the folder $(NAME).lv2 from Program Files/Common Files/LV2$(reset)"
endif

clean:
ifndef EXTRAQUIET
	@$(ECHO) ". ., clean up$(reset)"
endif
	$(QUIET)rm -f *.a  *.lib *.o *.so *.dll xresources.h
	$(QUIET)rm -rf ../bin

