SAMPLES=grid waypoints obstacles hexagonal

gslprompt:
	@echo "Graph Search Library(C), 1999-2003"

compile: gslprompt
	@if [ ! -d ./bin ]; then \
		echo "Creating bin directory..."; \
		mkdir ./bin; \
	fi
	@for X in $(SAMPLES); do \
		echo "Compiling $$X sample..."; \
		g++ ./samples/$$X/$$X.cpp -w -I./lib -o ./bin/$$X ; \
	done
run: compile
	@for X in $(SAMPLES); do \
		echo "Running $$X sample..."; \
		./bin/$$X; \
		echo "... out of $$X sample."; \
	done
clean: gslprompt
	@if [ -d ./bin ]; then \
		echo "Deleting bin directory..."; \
		rm -r ./bin; \
	fi
