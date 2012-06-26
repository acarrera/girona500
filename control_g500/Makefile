download_and_build: downloaded all

DATAFILE=data2.tar.gz
FOLDER = data2

downloaded:
	wget http://eia.udg.edu/~npalomer/files/$(DATAFILE) 
	tar xvf $(DATAFILE)	
	mv $(FOLDER) .. 
	rm $(DATAFILE)
	touch downloaded

include $(shell rospack find mk)/cmake.mk
