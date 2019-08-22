icl_dataset_folder = ~
icl_camera_parameters = 481.2,-480,320,240
icl_run_command = ./build/se_apps/se-denseslam-ofusion-main
icl_run_arguments = --input-file $(icl_dataset_folder)/scene.raw \
    --volume-size 6 --init-pose 0.5,0.5,0.5 --compute-size-ratio 2 \
    --integration-rate 1 --rendering-rate 1 --camera $(icl_camera_parameters)
run_command = $(icl_run_command)
run_arguments = $(icl_run_arguments)

all :
	mkdir -p build
	mkdir -p build/Release/
	cd build/Release/ && cmake -DCMAKE_BUILD_TYPE=Release \
		$(CMAKE_ARGUMENTS) ../..
	$(MAKE) -C build/Release/  $(MFLAGS) $(SPECIFIC_TARGET)


debug:
	mkdir -p build/
	mkdir -p build/Debug/
	cd build/Debug/ && cmake -DCMAKE_BUILD_TYPE=Debug \
		$(CMAKE_ARGUMENTS) ../..
	$(MAKE) -C build/Debug/ $(MFLAGS)

debug2:
	mkdir -p build/
	mkdir -p build/Debug2/
	cd build/Debug2/ && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
		$(CMAKE_ARGUMENTS) ../..
	$(MAKE) -C build/Debug2/ $(MFLAGS)

stats: 
	mkdir -p build/
	mkdir -p build/logs/
	cd build/ && cmake -DSTATS=ON ..
	$(MAKE) -C build $(MFLAG

install:
	cd build && make install

uninstall:
	cd build && make uninstall

#### DATA SET GENERATION ####

living_room_traj%_loop.raw : living_room_traj%_loop
	if test -x ./build/thirdparty/scene2raw ; then echo "..." ; else echo "do make before"; false ; fi
	./build/thirdparty/scene2raw living_room_traj$(*F)_loop living_room_traj$(*F)_loop.raw

living_room_traj%_loop : 
	mkdir $@
	cd $@ ; wget http://www.doc.ic.ac.uk/~ahanda/$@.tgz; tar xzf $@.tgz 

livingRoom%.gt.freiburg : 
	echo  "Download ground truth trajectory..."
	if test -x $@ ; then echo "Done" ; else wget http://www.doc.ic.ac.uk/~ahanda/VaFRIC/$@ ; fi

live.log : 
	./build/kfusion-qt-openmp $(live)

demo-ofusion:
	./build/kfusion-main-openmp --compute-size-ratio 2 --fps 0 --block-read False --input-file /data/ev314/data/living_room_traj2_frei_png/scene.raw --icp-threshold 1e-05 --mu 0.008 --init-pose 0.34,0.5,0.24 --integration-rate 1 --volume-size 5 -B 8 --tracking-rate 1 --volume-resolution 512 --pyramid-levels 10,5,4 --rendering-rate 1 -k 481.2,-480,320,240

demo-kfusion:
	./build/kfusion-main-openmp --compute-size-ratio 2 --fps 0 --block-read False --input-file /data/ev314/data/living_room_traj2_frei_png/scene.raw --icp-threshold 1e-05 --mu 0.1 --init-pose 0.34,0.5,0.24 --integration-rate 1 --volume-size 5 -B 8 --tracking-rate 1 --volume-resolution 512 --pyramid-levels 10,5,4 --rendering-rate 1 -k 481.2,-480,320,240


#### GENERAL GENERATION ####

doc :
	doxygen

clean :
	rm -rf build
cleanall : 
	rm -rf build
	rm -rf living_room_traj*_loop livingRoom*.gt.freiburg living_room_traj*_loop.raw
	rm -f *.log 
	rm -f doc


.PHONY : clean bench test all validate doc install uninstall

.PRECIOUS: living_room_traj%_loop livingRoom%.gt.freiburg living_room_traj%_loop.raw

.PHONY: run_icl
run_icl:
	$(icl_run_command) $(icl_run_arguments)

.PHONY: run
run:
	$(run_command) $(run_arguments)

.PHONY: run_gdb
run_gdb:
	gdb --args $(run_command) $(run_arguments)
