estimation: ekf.o
	g++ -g ekf.o -I/usr/include/eigen3 -o estimation

#rotation_vector_utils.o: rotation_vector_utils.cpp rotation_vector_utils.h
#	g++ -o rotation_vector_utils rotation_vector_utils.cpp

#matrix_utils.o: matrix_utils.cpp matrix_utils.h
#	g++ -o matrix_utils matrix_utils.cpp matrix_utils.h

ekf.o: ekf.cpp newEKF.h
	g++ -g -c ekf -I/usr/include/eigen3 ekf.cpp

clean:
	rm *.o 