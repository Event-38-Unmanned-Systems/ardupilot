SCRIPT_DIR="$(dirname "$(realpath "$0")")"
cd $SCRIPT_DIR/modules/mavlink
git apply $SCRIPT_DIR/mavlink_submodule.patch
