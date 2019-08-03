#!/bin/bash

FILE=$1

program_counter=$(cat $FILE | grep "pc: " | awk '{print $2}' )
x0=$(cat $FILE | grep "x\[ 0\]:" | awk '{print $3}')
x1=$(cat $FILE | grep "x\[ 1\]:" | awk '{print $3}')
x2=$(cat $FILE | grep "x\[ 2\]:" | awk '{print $3}')
x3=$(cat $FILE | grep "x\[ 3\]:" | awk '{print $3}')
x4=$(cat $FILE | grep "x\[ 4\]:" | awk '{print $3}')
x5=$(cat $FILE | grep "x\[ 5\]:" | awk '{print $3}')
x6=$(cat $FILE | grep "x\[ 6\]:" | awk '{print $3}')
x7=$(cat $FILE | grep "x\[ 7\]:" | awk '{print $3}')
x8=$(cat $FILE | grep "x\[ 8\]:" | awk '{print $3}')
x9=$(cat $FILE | grep "x\[ 9\]:" | awk '{print $3}')
x10=$(cat $FILE | grep "x\[10\]:" | awk '{print $2}')
x11=$(cat $FILE | grep "x\[11\]:" | awk '{print $2}')
x12=$(cat $FILE | grep "x\[12\]:" | awk '{print $2}')
x13=$(cat $FILE | grep "x\[13\]:" | awk '{print $2}')
x14=$(cat $FILE | grep "x\[14\]:" | awk '{print $2}')
x15=$(cat $FILE | grep "x\[15\]:" | awk '{print $2}')
x16=$(cat $FILE | grep "x\[16\]:" | awk '{print $2}')
x17=$(cat $FILE | grep "x\[17\]:" | awk '{print $2}')
x18=$(cat $FILE | grep "x\[18\]:" | awk '{print $2}')
x19=$(cat $FILE | grep "x\[19\]:" | awk '{print $2}')
x20=$(cat $FILE | grep "x\[20\]:" | awk '{print $2}')
x21=$(cat $FILE | grep "x\[21\]:" | awk '{print $2}')
x22=$(cat $FILE | grep "x\[22\]:" | awk '{print $2}')
x23=$(cat $FILE | grep "x\[23\]:" | awk '{print $2}')
x24=$(cat $FILE | grep "x\[24\]:" | awk '{print $2}')
x25=$(cat $FILE | grep "x\[25\]:" | awk '{print $2}')
x26=$(cat $FILE | grep "x\[26\]:" | awk '{print $2}')
x27=$(cat $FILE | grep "x\[27\]:" | awk '{print $2}')
x28=$(cat $FILE | grep "x\[28\]:" | awk '{print $2}')
x29=$(cat $FILE | grep "x\[29\]:" | awk '{print $2}')
x30=$(cat $FILE | grep "x\[30\]:" | awk '{print $2}')
x31=$(cat $FILE | grep "x\[31\]:" | awk '{print $2}')

#echo "$tmp" | wc -l
paste -d " " <(printf %s "$program_counter") \
	<(printf %s "$x0") \
	<(printf %s "$x1") \
	<(printf %s "$x2") \
	<(printf %s "$x3") \
	<(printf %s "$x4") \
	<(printf %s "$x5") \
	<(printf %s "$x6") \
	<(printf %s "$x7") \
	<(printf %s "$x8") \
	<(printf %s "$x9") \
	<(printf %s "$x10") \
	<(printf %s "$x11") \
	<(printf %s "$x12") \
	<(printf %s "$x13") \
	<(printf %s "$x14") \
	<(printf %s "$x15") \
	<(printf %s "$x16") \
	<(printf %s "$x17") \
	<(printf %s "$x18") \
	<(printf %s "$x19") \
	<(printf %s "$x20") \
	<(printf %s "$x21") \
	<(printf %s "$x22") \
	<(printf %s "$x23") \
	<(printf %s "$x24") \
	<(printf %s "$x25") \
	<(printf %s "$x26") \
	<(printf %s "$x27") \
	<(printf %s "$x28") \
	<(printf %s "$x29") \
	<(printf %s "$x30") \
	<(printf %s "$x31")
