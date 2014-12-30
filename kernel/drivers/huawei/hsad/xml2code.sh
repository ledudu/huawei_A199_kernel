AUTO_GENERATE=auto-generate
PWD=`pwd`

#mkdir auto-generate
if [ -d ${AUTO_GENERATE} ]
then :
else
	mkdir ${AUTO_GENERATE}
fi
cd ${AUTO_GENERATE}
perl ./../parse_product_id.pl ./../../../../../device/hisi/customize/hsad/product_boardid.xml
