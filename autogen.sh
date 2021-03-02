#!/bin/sh
bs_dir=$(cd "$(dirname "$0")"; pwd)

autoreconf -fi "${bs_dir}"

if test -n "$1" && test -z "$NOCONFIGURE" ; then
	echo 'Configuring...'
	"$bs_dir"/configure "$@"
fi

# How to configure for standalone compile
#   $> source /opt/poky/2.5.1_64/environment-setup-aarch64-poky-linux 
#   $> ./autogen.sh
#   $> ./configure --host=aarch64-poky-linux --with-system-jansson --enable-btc08

