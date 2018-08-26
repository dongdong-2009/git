HOSTCC=gcc

all: conf mconf

LIBS=-lncurses
#下面这句的目的是什么
ifeq (/usr/include/ncursesw/ncurses.h, $(wildcard /usr/include/ncursesw/ncurses.h))
	HOSTNCURSES += -I/usr/include/ncursesw -DCURSES_LOC="<ncurses.h>"
	LIBS=-lncursesw
else
	ifeq (/usr/include/ncurses/ncurses.h, $(wildcard /usr/include/ncurses/ncurses.h))
		HOSTNCURSES += -I/usr/include/ncurses -DCURSES_LOC="<ncurses.h>"
	else
		ifeq (/usr/include/ncurses/curses.h, $(wildcard /usr/include/ncurses/curses.h))
			HOSTNCURSES += -I/usr/include/ncurses -DCURSES_LOC="<ncurses/curses.h>"
		else
			ifeq (/usr/local/include/ncurses/ncurses.h, $(wildcard /usr/local/include/ncurses/ncurses.h))
				HOSTCFLAGS += -I/usr/local/include/ncurses -DCURSES_LOC="<ncurses.h>"
			else
				ifeq (/usr/local/include/ncurses/curses.h, $(wildcard /usr/local/include/ncurses/curses.h))
					HOSTCFLAGS += -I/usr/local/include/ncurses -DCURSES_LOC="<ncurses/curses.h>"
				else
					ifeq (/usr/include/ncurses.h, $(wildcard /usr/include/ncurses.h))
						HOSTNCURSES += -DCURSES_LOC="<ncurses.h>"
					else
						HOSTNCURSES += -DCURSES_LOC="<curses.h>"
					endif
				endif
			endif
		endif
	endif
endif


lxdialog := lxdialog/checklist.o lxdialog/util.o lxdialog/inputbox.o
lxdialog += lxdialog/textbox.o lxdialog/yesno.o lxdialog/menubox.o

SHARED_SRC=zconf.tab.c
CONF_SRC=conf.c
MCON_SRC=mconf.c $(lxdialog) $(SHARED_SRC)

SHARED_DEPS:=lkc.h lkc_proto.h expr.h
CONF_OBJS=$(patsubst %.c, %.o, $(CONF_SRC))
MCONF_OBJS=$(patsubst %.c, %.o, $(MCON_SRC))
SHARED_OBJS=$(patsubst %.c, %.o, $(SHARED_SRC))

conf: $(CONF_OBJS) $(SHARED_OBJS)
	@$(HOSTCC) $^ -o $@

mconf: $(MCONF_OBJS) $(SHARED_OBJS)
	@$(HOSTCC) $^ -o $@ $(LIBS)
	
$(CONF_OBJS): %.o : %.c $(SHARED_DEPS)
	@$(HOSTCC) $(HOSTCFLAGS) -I. -c $< -o $@
	
$(MCONF_OBJS): %.o : %.c $(SHARED_DEPS)
	@$(HOSTCC) $(HOSTCFLAGS) $(HOSTNCURSES) -I. -c $< -o $@
	
zconf.tab.c : zconf.hash.c zconf.lex.c
	@cp zconf.tab.c_shipped zconf.tab.c

zconf.hash.c :
	@cp zconf.hash.c_shipped zconf.hash.c

zconf.lex.c :
	@cp zconf.lex.c_shipped zconf.lex.c
	
clean :
	@rm -rf *.o conf mconf zconf.tab.c zconf.hash.c zconf.lex.c
	@rm -rf lxdialog/*.o
