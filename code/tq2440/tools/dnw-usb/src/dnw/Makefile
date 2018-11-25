
dnw : dnw.c
	@gcc -g -o $@ $^
	
clean :
	@rm -rf dnw *.o

install : dnw
	@sudo cp $^ /usr/bin
