#!/bin/sh
aclocal
libtoolize -c -f
automake -ca
autoconf
