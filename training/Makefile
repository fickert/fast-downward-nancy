# This makefile builds all generators that need to be compiled.

all: generators/blocksworld generators/elevators generators/rovers/rovers generators/satellite/satellite generators/tidybot/tidybot-1.0.1.jar generators/tidybot/tidy generators/visitall/visitall

generators/blocksworld: generators/blocksworld/bwstates generators/blocksworld/topddl

generators/blocksworld/bwstates: generators/blocksworld/bwstates.c
	$(CC) $(CFLAGS) -DSZ=1000 $^ -o $@ -lm

generators/blocksworld/topddl: generators/blocksworld/2pddl.c
	$(CC) $(CFLAGS) $^ -o $@ -lm

generators/elevators: generators/elevators/generate generators/elevators/topddl

generators/elevators/generate: generators/elevators/generate_elevators.c
	$(CC) $(CFLAGS) $^ -o $@ -lm

generators/elevators/topddl: generators/elevators/generate_pddl.c
	$(CC) $(CFLAGS) $^ -o $@ -lm

generators/rovers/rovers: generators/rovers/rovers.cc
	$(CXX) $(CFLAGS) $^ -o $@

generators/satellite/satellite: generators/satellite/satellite.cc
	$(CXX) $(CFLAGS) $^ -o $@

generators/tidybot/tidybot-1.0.1.jar: generators/tidybot/src/tidybot/core.clj
	cd generators/tidybot/ && ./lein compile && \
	  ./lein uberjar && \
	  mv ./target/tidybot-1.0.1-snapshot-standalone.jar ./tidybot-1.0.1.jar && \
	  rm -rf ./target && \
	  cd ../../

generators/tidybot/tidy: generators/tidybot/tidy.c
	$(CC) $(CFLAGS) $^ -o $@

generators/visitall: generators/visitall/visitall.c
	$(CC) $(CFLAGS) $^ -o $@

.PHONY: clean
clean:
	rm -rf generators/blocksworld/bwstates \
	  generators/blocksworld/topddl \
	  generators/rovers/rovers \
	  generators/visitall/visitall \
	  generators/satellite/satellite \
	  generators/tidybot/tidybot-1.0.1.jar \
	  generators/tidybot/tidy \
	  generators/elevators/generate \
	  generators/elevators/topddl \
	  instances/*
