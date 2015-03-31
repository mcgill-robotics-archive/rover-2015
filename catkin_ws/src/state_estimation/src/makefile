estimation: ekfTest.o
	g++ -g ekfTest.o -I/usr/include/eigen3 -o estimation

#rotation_vector_utils.o: rotation_vector_utils.cpp rotation_vector_utils.h
#	g++ -o rotation_vector_utils rotation_vector_utils.cpp

#matrix_utils.o: matrix_utils.cpp matrix_utils.h
#	g++ -o matrix_utils matrix_utils.cpp matrix_utils.h

ekfTest.o: ekfTest.cpp newEKF.h
	g++ -g -c ekfTest.cpp  newEKF.h -I/usr/include/eigen3 

clean:
	rm *.o 