#/bin/bash


ROOT=`pwd`
echo $ROOT

ALLDIR=`find ./ -type d`

for i in $ALLDIR
do
	cd $i
	rm -fr *.uvproj *.uvopt *.sgxx *.ewp rvmdk-* ewarm-* ccs-* *.eww *.sgxw *.uvmpw ccs ewarm rvmdk sourcerygxx *_ccs.cmd *_sourcerygxx.ld *.ewd *.icf *.sct *_ewarm.c *_rvmdk.S *_ccs.c sourcerygxx* codered startup_codered.c cr_project.xml *_codered.ld
	cd $ROOT
done
