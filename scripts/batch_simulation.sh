#!/bin/bash
# 2015-09-03
# v 0.6a
# Created by J. Cappelletto

#######################################################################################################################
# Parsing method extracted from http://wiki.bash-hackers.org/howto/getopts_tutorial
#######################################################################################################################
PATH_CTD='data/ctd/'
PATH_TRANSECT='data/transects/'
DIAMETER_LIST=($(seq 0.014 0.002 0.020))
CREATE_FOLDER=false

echo -e "CTD path:\t $PATH_CTD" >&2
echo -e "Transects path:\t $PATH_TRANSECT" >&2
echo -e "Ball diameters:\t ${DIAMETER_LIST[@]}" >&2

# Retrieves the list of all video files with $VIDEO_FMT extension

shopt -s nullglob

CTD_LIST=$(find $PATH_CTD -name '_CTD*.csv')
TRANSECT_LIST=$(find $PATH_TRANSECT -name 'T*.csv')

echo $CTD_LIST
echo $TRANSECT_LIST

for transect in $TRANSECT_LIST; do
	echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
#	echo "Transect: "$transect
	echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
	transect_id=$(basename "$transect")	#extract file name with extension
	transect_id=${transect_id%.csv}		#strip extension for file name, and use it as ID
	echo $transect_id
	for ctd in $CTD_LIST; do
		echo "..................."
#		echo "CTD profile: "$ctd
		echo "..................."
		ctd_id=$(basename "$ctd")	#extract file name with extension
		ctd_id=${ctd_id%.csv}		#strip extension for file name, and use it as ID
		echo $ctd_id
		for diameter in ${DIAMETER_LIST[@]}; do
			#echo "Diameter: "$diameter
			# we have to replace the COMMA form the diameter list to DOT
			diameter=${diameter//,/.}
			outputfile="_e0.03_d"$diameter"_c"$ctd_id"_t"$transect_id
			echo ">> Calling driftcam_simulator for: "$outputfile
			$(python driftcam_simulator.py --transect $transect --ctd $ctd --ballast $diameter --output $outputfile)
		done
	done
done
