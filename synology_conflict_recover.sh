#! /bin/bash

for file in `find . -name "*_JJHui-Macbook-Air.local_Mar-23-*"`; do
	mv $file "${file%%_JJHui-Macbook-Air.local_Mar-23-*}.${file##*.}"
done
