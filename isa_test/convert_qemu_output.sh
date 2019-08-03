#!/bin/bash

FILE="$1"

program_counter=$(cat $FILE | grep " pc      " | awk '{print $2}' )
first=$(cat $FILE | grep zero | grep ra | grep sp | grep gp | awk '{print $2 " " $4 " " $6 " " $8}')
second=$(cat $FILE | grep tp | grep t0 | grep t1 | grep t2 | awk '{print $2 " " $4 " " $6 " " $8}')
third=$(cat $FILE | grep s0 | grep s1 | grep a0 | grep a1 | awk '{print $2 " " $4 " " $6 " " $8}')
fourth=$(cat $FILE | grep a2 | grep a3 | grep a4 | grep a5 | awk '{print $2 " " $4 " " $6 " " $8}')
fifth=$(cat $FILE | grep a6 | grep a7 | grep s2 | grep s3 | awk '{print $2 " " $4 " " $6 " " $8}')
sixth=$(cat $FILE | grep s4 | grep s5 | grep s6 | grep s7 | awk '{print $2 " " $4 " " $6 " " $8}')
seventh=$(cat $FILE | grep s8 | grep s9 | grep s10 | grep s11 | awk '{print $2 " " $4 " " $6 " " $8}')
eighth=$(cat $FILE | grep t3 | grep t4 | grep t5 | grep t6 | awk '{print $2 " " $4 " " $6 " " $8}')

#echo "$tmp" | wc -l
paste -d " " <(printf %s "$program_counter") \
	<(printf %s "$first") \
	<(printf %s "$second") \
	<(printf %s "$third") \
	<(printf %s "$fourth") \
	<(printf %s "$fifth") \
	<(printf %s "$sixth") \
	<(printf %s "$seventh") \
	<(printf %s "$eighth")
