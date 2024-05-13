# aa:
# 	gcc sub.c add.c multi.c calc.c -o aa

# c:add.o sub.o multi.o calc.o
# 	gcc sub.o add.o multi.o calc.o -o c
# add.o:add.c
# 	gcc -c add.c -o add.o

# sub.o:sub.c
# 	gcc -c sub.c -o sub.o

# multi.o:multi.c
# 	gcc -c multi.c -o multi.o

# clean:
# 	rm -rf *.o c
Obj=add.o sub.o multi.o calc.o
Tab=c

# $(Tab):$(Obj)
# 	gcc $(Obj) -o $(Tab)
# add.o:add.c
# 	gcc -c add.c -o add.o

# sub.o:sub.c
# 	gcc -c sub.c -o sub.o

# multi.o:multi.c
# 	gcc -c multi.c -o multi.o

# clean:
# 	rm -rf *.o c


.PHONY:clean c 

$(Tab):$(Obj)
	gcc $(Obj) -o $(Tab)
add.o:add.c
	gcc -c $^ -o $@

sub.o:sub.c
	gcc -c $^ -o $@

multi.o:multi.c
	gcc -c $^ -o $@

clean:
	rm -rf *.o c