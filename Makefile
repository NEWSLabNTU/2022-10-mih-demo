.PHONY: build clean pull_ros_dependencies doc

build: pull_ros_dependencies
	@echo 'Building this project' >&2

	@if [ -z "$$ROS_DISTRO" ] ; then \
		echo 'Error: Did you source setup.{sh,zsh,...}?'; \
		exit 1; \
	fi

	colcon build --symlink-install \
		--cmake-args -DCMAKE_BUILD_TYPE=Release \
		--cargo-args --release

pull_ros_dependencies: deps.repos
	@echo 'Pulling ROS dependencies' >&2
	@vcs import src < deps.repos && \
	vcs pull src < deps.repos

doc: README.html

README.html: README.md doc/architecture.dot.png doc/legend.dot.png
	grip --export README.md

doc/%.dot.png: doc/%.dot
	dot -Tpng $< > $@


clean:
	rm -f README.html doc/architecture.png
	cargo clean
