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

doc:
	cd doc && dot -Tpng ARCHITECTURE.dot > ARCHITECTURE.png

clean:
	cargo clean
