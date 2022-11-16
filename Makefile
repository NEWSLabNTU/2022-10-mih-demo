.PHONY: build clean build_ros_dependencies doc

build:
	@echo 'Building this project' >&2
	@. repos/install/setup.sh && \
	cargo build --release --all-targets

build_ros_dependencies:
	@echo 'Pulling ROS dependencies' >&2
	@mkdir -p repos && \
	vcs import repos < dependencies.repos && \
	vcs pull repos < dependencies.repos

	@echo 'Building ROS dependencies' >&2
	@cd repos && \
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

doc: README.html

README.html: README.md doc/architecture.dot.png doc/legend.dot.png
	grip --export README.md

doc/%.dot.png: doc/%.dot
	dot -Tpng $< > $@


clean:
	rm -f README.html doc/architecture.png
	cargo clean
