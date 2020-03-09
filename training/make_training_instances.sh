#!/usr/bin/env bash

# This script compiles all instance generators that need to be
# compiled.  It then generates instances.  The parameters of the
# instance generators were chosen to be as close to the ones used in
# the IPC.  Since we don't want exactly the same instances though, we
# use a different seed for those generators that accept a seed
# parameter.
# If this script is called without any parameters, it generates
# instances for all domains.  To generate instances for a subset of
# domains, provide their lower-case names as command line parameters.

root=${PWD}

echo "Compiling generators"
make -j$(nproc) || exit 1

# root directory to put all generated instances
outdir="${root}/instances"
mkdir -p "${outdir}"

declare -A selections
if [ "$#" == 0 ]; then
	selections["all"]=1
else
	for i in "${@}"; do
		selections[${i#*-}]=1
	done
fi


if [[ -v "selections[all]" || -v "selections[barman]" ]]; then
	echo "Generating barman instances"
	# shots, ingredients, cocktails
	shts=( 4 5 6 8 9 4 5 6 7 8 )
	igrs=( 3 3 3 3 3 3 3 3 3 3 )
	cktl=( 3 4 5 6 7 5 6 8 9 9 )
	len="${#shts[@]}"
	mkdir -p "${outdir}/barman"
	cp "generators/barman/domain.pddl" "${outdir}/barman/domain.pddl"
	for (( i=0; i<$len; ++i )) ; do
		j=1
		while :; do
			p1=${cktl[${i}]}
			p2=${igrs[${i}]}
			p3=${shts[${i}]}
			# In 2011, generate 4 instances.
			# In 2014, generate 2 instances for 8 cocktail instances, and 3 for all others.
			if (( ${i} < 5 )) ; then
				if (( ${j} > 4 )) ; then
					break
				fi
				n="2011"
			else
				if (( ${p1} == 8 && ${j} > 2 || ${j} > 3 )) ; then
					break
				fi
				n="2014"
			fi
			./generators/barman/barman.py ${p1} ${p2} ${p3} > "${outdir}/barman/p${n}-${p1}-${p2}-${p3}-${j}.pddl"
			((++j))
		done
	done
fi

if [[ -v "selections[all]" || -v "selections[blocks]" ]]; then
	echo "Generating blocksworld instances"
	gen_blks() {
		# topddl doesn't work with a pipe unfortunately
		./generators/blocksworld/bwstates -s 2 -n $1 > ./generators/blocksworld/STATES \
			&& ./generators/blocksworld/topddl -d ./generators/blocksworld/STATES -n $1 \
			&& rm ./generators/blocksworld/STATES
	}
	mkdir -p "${outdir}/blocksworld"
	cp "generators/blocksworld/domain.pddl" "${outdir}/blocksworld/domain.pddl"
	for (( i=4; i<18; ++i )) ; do
		gen_blks ${i} > "${outdir}/blocksworld/p-${i}-1.pddl"
		(( i >= 17 )) && continue
		gen_blks ${i} > "${outdir}/blocksworld/p-${i}-2.pddl"
		(( i >= 12 )) && continue
		gen_blks ${i} > "${outdir}/blocksworld/p-${i}-3.pddl"
	done
fi

if [[ -v "selections[all]" || -v "selections[elevators]" ]]; then
	echo "Generating elevators instances"
	mkdir -p "${outdir}/elevators"
	cp "generators/elevators/domain.pddl" "${outdir}/elevators/domain.pddl"
	# floors, passengers, slow elevators, fast elevators, slow capacity, fast capacity, area size
	#ipc2008                                                                                   #ipc2011
	flrs=( 10 10 10 10 10 10 10 10 10 10 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12  12 12 12  8 12  8 12  8 12 12 12  8  8  8  8 12 12 12 12 12  )
	psgs=(  3  3  4  4  5  5  6  6  7  7  3  3  4  4  5  5  6  6  7  7  3  3  4  4  5  5  6  6  7  7   3  3  3  4  4  4  5  5  3  4  6  5  6  6  7  5  6  6  4  4  )
	sels=(  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1   1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  )
	fels=(  1  2  1  2  1  2  1  2  1  2  1  2  1  2  1  2  1  2  1  2  1  2  1  2  1  2  1  2  1  2   1  1  2  1  1  2  2  1  2  2  2  2  1  2  1  1  1  2  1  2  )
	scap=(  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2   2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  )
	fcap=(  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3   3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  3  )
	arsz=(  4  4  4  4  4  4  4  4  4  4  6  6  6  6  6  6  6  6  6  6  4  4  4  4  4  4  4  4  4  4   6  4  6  4  6  4  4  4  4  6  4  4  4  4  4  6  6  6  4  4  )
	len="${#flrs[@]}"
	for (( i=0; i<$len; ++i )) ; do
		n=$(( $i < 30 ? "2008" : "2011" ))
		p1=${flrs[${i}]}
		p2=${psgs[${i}]}
		p3=${sels[${i}]}
		p4=${fels[${i}]}
		p5=${scap[${i}]}
		p6=${fcap[${i}]}
		p7=${arsz[${i}]}
		prefix="p${p1}_${p2}_1"
		./generators/elevators/generate ${p2} ${p2} 1 1 1 ${p1} ${p7} ${p4} ${p3} ${p6} ${p5} > /dev/null \
			&& ./generators/elevators/topddl ${p1} ${p1} 1 ${p2} ${p2} 1 1 1 > /dev/null \
			&& rm "${prefix}.txt" \
			&& mv "${prefix}.pddl" "${outdir}/elevators/p${n}-${p1}-${p2}-${p3}-${p4}-${p5}-${p6}-${p7}.pddl"
	done
fi

if [[ -v "selections[all]" || -v "selections[parking]" ]]; then
	echo "Generating parking instances"
	#ipc 2011                                                           #ipc 2014
	crbs=(  7  7  8  8  8  8  9  9  9  9 10 10 10 10 11 11 11 11 12 12   7  7  7  7  8  8  8  8  9  9  9  9 10 10 10 10 11 11 11 11)
	cars=( 12 12 14 14 14 14 16 16 16 16 18 18 18 18 20 20 20 20 22 22  12 12 12 12 14 14 14 14 16 16 16 16 18 18 18 18 20 20 20 20)
	len="${#cars[@]}"
	mkdir -p "${outdir}/parking"
	for (( i=0; i<$len; ++i )) ; do
		n=$(( $i < 20 ? "2011" : "2014" ))
		p1=${crbs[${i}]}
		p2=${cars[${i}]}
		./generators/parking/parking.pl "_" "${p1}" "${p2}" "seq" > "${outdir}/parking/p${n}-$((i+1))-${p1}-${p2}.pddl"
	done
fi

if [[ -v "selections[all]" || -v "selections[rovers]" ]]; then
	echo "Generating rovers instances"
	# rovers, waypoints, objectives, cameras, number of goals
	rovs=( 1  1  2  2  2  2  3  4  4  4  4  4  4  4  4  4  6  6  6  8)
	wpts=( 4  4  4  4  4  6  6  6  7  7  8  8  9 10 11 12 15 20 20 25)
	objs=( 2  2  2  3  3  2  2  3  3  4  3  4  4  4  5  5  6  7  8  8)
	cams=( 1  2  2  3  3  3  2  4  5  6  4  4  5  5  4  4  7  7  7  7)
	ngls=( 1  1  1  1  2  3  2  3  4  7  6  3  5  4  6  7  7  5  5  7)
	seed=( 202020 404040 9876 )
	len="${#rovs[@]}"
	mkdir -p "${outdir}/rovers"
	cp "generators/rovers/domain.pddl" "${outdir}/rovers/domain.pddl"
	for s in ${seed[@]} ; do
		for (( i=0; i<$len; ++i )) ; do
			p1=${rovs[${i}]}
			p2=${wpts[${i}]}
			p3=${objs[${i}]}
			p4=${cams[${i}]}
			p5=${ngls[${i}]}
			name="rovers-${p1}-${p2}-${p3}-${p4}-${p5}-${s}.pddl"
			./generators/rovers/rovers ${s} ${p1} ${p2} ${p3} ${p4} ${p5} > "${outdir}/rovers/${name}"
		done
	done
fi

if [[ -v "selections[all]" || -v "selections[satellites]" ]]; then
	echo "Generating satellites instances"
	# satellites, max instruments/satellite, modes, targets, observations
	sats=( 1 1 2 2 3 3 4 4 5 5  5  5  5  6  8  10 12 5  5  5 )
	inst=( 1 3 3 3 3 3 3 3 3 3  3  3  3  3  3  3  3  5  8  10 )
	mods=( 3 3 3 3 3 4 4 4 5 5  5  5  5  5  5  5  5  5  8  10 )
	targ=( 3 4 4 4 4 5 5 6 6 7  9  12 13 12 10 10 10 10 10 10 )
	obsv=( 4 4 4 6 6 6 7 9 9 10 11 13 17 13 15 15 15 15 15 15 )
	seed=( 202020 404040 9876 )
	len="${#sats[@]}"
	mkdir -p "${outdir}/satellite"
	cp "generators/satellite/domain.pddl" "${outdir}/satellite/domain.pddl"
	for s in ${seed[@]} ; do
		for (( i=0; i<$len; ++i )) ; do
			p1=${sats[${i}]}
			p2=${inst[${i}]}
			p3=${mods[${i}]}
			p4=${targ[${i}]}
			p5=${obsv[${i}]}
			name="satellite-${p1}-${p2}-${p3}-${p4}-${p5}-${s}.pddl"
			./generators/satellite/satellite ${s} ${p1} ${p2} ${p3} ${p4} ${p5} > "${outdir}/satellite/${name}"
		done
	done
fi

if [[ -v "selections[all]" || -v "selections[termes]" ]]; then
	echo "Generating termes instances"
	mkdir -p "${outdir}/termes"
	cp "./generators/termes/domain.pddl" "${outdir}/termes/domain.pddl"
	mkdir -p "./generators/termes/instances/" "./generators/termes/plans/"
	seeds=( 1 2021 202020 )
	len="${#seeds[@]}"
	for (( i=0; i<$len; ++i )) ; do
		./generators/termes/generate-ipc-instances-random-towers.py -s ${seeds[${i}]} \
			&& ./generators/termes/select-ipc-instances.py |xargs -i{} mv {} "${outdir}/termes/"
		:;
	done
	rm -rf "./generators/termes/instances/" "./generators/termes/plans/"
fi

if [[ -v "selections[all]" || -v "selections[tidybot]" ]]; then
	# Note: The generator for tidybot is weird beast.
	# http://www.plg.inf.uc3m.es/ipc2011-deterministic/DomainsSequential.html#Tidybot
	# The input parameters are in no obvious relation to the
	# output instance, because the generator often tries to place
	# objects in a few positions and just discards them if it
	# fails to do so.  Furthermore, the positions where objects
	# can be placed at all are a pretty weird.  I'm not sure if
	# those are bugs or actually intended behavior.  In any case,
	# this is the generator that was used in the IPC, so we use it
	# as well.
	echo "Generating tidybot instances"
	# world-size, tables, cupboards, min surface size, max surface size, cupboard size
	wrld=( 5 6 6 6 7 7 7 7 8 8 8 8 8 9 9 9 9 9 9 9 )
	ntbl=( 0 0 0 0 0 0 0 0 0 0 0 0 0 3 2 5 4 3 4 5 )
	ncup=( 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 )
	smin=( 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 )
	smax=( 1 1 2 3 1 2 3 4 1 2 3 4 5 1 2 2 3 4 4 5 )
	scup=( 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 )
	seed=( 1337 1851 2020 4040 )
	len="${#wrld[@]}"
	mkdir -p "${outdir}/tidybot"
	cp "generators/tidybot/domain.pddl" "${outdir}/tidybot/domain.pddl"
	for s in ${seed[@]}; do
		for (( i=0; i<$len; ++i )) ; do
			p1=${wrld[${i}]}
			p2=${ntbl[${i}]}
			p3=${ncup[${i}]}
			p4=${smin[${i}]}
			p5=${smax[${i}]}
			p6=${scup[${i}]}
			name="tidybot-${p1}-${p2}-${p3}-${p4}-${p5}-${p6}-${s}.pddl"
			java -jar ./generators/tidybot/tidybot-1.0.1.jar ${p1} ${p2} ${p3} ${p4} ${p5} ${p6} ${s} > "${outdir}/tidybot/${name}"
		done
	done
fi

if [[ -v "selections[all]" || -v "selections[transport]" ]]; then
	echo "Generating transport instances"
	# cities, nodes, size, degrees, min distance, number of trucks, number of packages
	#ipc 2008                                                                                  # ipc2011                                                    # ipc2014
	cits=( 1  1  1  1  1  1  1  1  1  1  2  2  2  2  2  2  2  2  2  2  3  3  3  3  3  3  3  3  3  3   3  1  2  2  3  1  1  1  1  1  2  2  1  1  3  3  1  2  3  1   1  1  1  1  1  1  2  2  2  2  2  2  3  3  3  3  3  3  3  3 )
	nods=( 3  6  9 12 15 18 21 24 27 30  2  4  6  8 10 12 14 16 18 20  1  2  3  4  5  6  7  8  9 10   3  9  4  6  4 12 13 13 14 14  7  7 15 15  5  5 15  8  5 18   5 10 12 15 18 20  5 10 12 15 18 20  5 10 12 15 18 20 20 20 )
	size=1000
	degr=( 2  2  3  3  3  4  4  4  4  4  2  2  3  3  3  4  4  4  4  4  0  1  2  2  3  4  4  4  4  4   2  3  2  3  2  3  2  3  2  3  4  4  2  3  4  4  3  3  3  4   2  2  3  3  4  4  2  2  3  3  4  4  2  2  3  3  4  4  4  4 )
	mind=100
	ntrk=( 2  2  3  3  3  4  4  4  4  4  2  2  3  3  3  4  4  4  4  4  2  2  3  3  3  4  4  4  4  4   2  2  2  2  2  2  2  3  2  3  2  3  2  3  2  3  2  2  2  3   2  2  2  3  3  3  2  2  2  3  3  3  2  2  2  3  3  3  3  3 )
	npkg=( 2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31   4  4  3  4  5  5  6  6  7  7  8  8  8  8  9  9  6  5  6  7   4  4  5  5  6  6  4  4  5  5  6  6  4  4  5  5  6  6  7  7 )
	gens=( "" "city-generator.py" "two-cities-generator.py" "three-cities-generator.py" )
	# The generator can sometimes randomly fail to create a graph.  But this seed succeeds for all inputs.
	seed=2022
	len="${#cits[@]}"
	mkdir -p "${outdir}/transport"
	cp "./generators/transport/domain.pddl" "${outdir}/transport/domain.pddl"
	for (( i=0; i<$len; ++i )) ; do
		n=$(( $i < 50 ? ( $i < 30 ? "2008" : "2011" ) : "2014" ))
		p1=${cits[${i}]}
		gen=${gens[${p1}]}
		p2=${nods[${i}]}
		p3=${degr[${i}]}
		p4=${ntrk[${i}]}
		p5=${npkg[${i}]}
		./generators/transport/${gen} ${p2} ${size} ${p3} ${mind} ${ntrk} ${npkg} ${seed} > "${outdir}/transport/p${n}-$((i+1)).pddl"
	done
fi

if [[ -v "selections[all]" || -v "selections[visitall]" ]]; then
	echo "Generating visitall instances"
	mkdir -p "${outdir}/visitall"
	cp "generators/visitall/domain.pddl" "${outdir}/visitall/domain.pddl"
	for (( i=2; i<16; ++i )) ; do
		./generators/visitall/visitall -s   2021 -n ${i} -r 1.0 > "${outdir}/visitall/visitall-${i}-10-2021.pddl"
		./generators/visitall/visitall -s 202020 -n ${i} -r 0.3 > "${outdir}/visitall/visitall-${i}-03-202020.pddl"
		./generators/visitall/visitall -s 404040 -n ${i} -r 0.3 > "${outdir}/visitall/visitall-${i}-03-404040.pddl"
	done
fi
