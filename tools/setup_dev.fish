#!/usr/bin/env fish

set -l ros_setup /opt/ros/lyrical/setup.fish
if not test -f $ros_setup
    echo "ROS 2 Lyrical setup not found: $ros_setup" >&2
    return 1
end

source $ros_setup

set -l repo_root (path resolve (dirname (status filename))/..)
set -gx COLCON_DEFAULTS_FILE $repo_root/colcon.defaults.yaml
set -gx ROS_LOG_DIR $repo_root/log/ros

# The managed development environment may expose an immutable empty .git
# placeholder. Fall back to repository metadata stored beside it. Normal
# clones continue using their standard .git directory.
if not git --git-dir=$repo_root/.git rev-parse --git-dir >/dev/null 2>&1
    if test -d $repo_root/.git-data
        set -gx GIT_DIR $repo_root/.git-data
        set -gx GIT_WORK_TREE $repo_root
    end
end

set -l workspace_install $repo_root/install
if test -d $workspace_install
    # colcon currently has no installed fish-shell extension. Reproduce the
    # essential isolated-workspace environment for each installed package.
    for package_name in (colcon list --base-paths $repo_root/src --topological-order --names-only 2>/dev/null)
        set -l package_prefix $workspace_install/$package_name
        if not test -d $package_prefix
            continue
        end
        fish_add_path --prepend --global $package_prefix/bin $package_prefix/lib/$package_name
        set -gx AMENT_PREFIX_PATH $package_prefix $AMENT_PREFIX_PATH
        set -gx CMAKE_PREFIX_PATH $package_prefix $CMAKE_PREFIX_PATH
        set -gx LD_LIBRARY_PATH $package_prefix/lib $LD_LIBRARY_PATH
        for python_packages in $package_prefix/lib/python*/site-packages
            if test -d $python_packages
                set -gx PYTHONPATH $python_packages $PYTHONPATH
            end
        end
    end
end

set -gx AUV_WS $repo_root
echo "AUV development environment ready: $AUV_WS (ROS_DISTRO=$ROS_DISTRO)"
