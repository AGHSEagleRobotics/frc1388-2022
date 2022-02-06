mkdir -p $(dirname $1)
cat <<EOF > $1
package frc.robot;
public class BuildInfo {
  public static final String GIT_VERSION = "$(git log -1 --pretty=format:%h)";
  public static final String GIT_BRANCH = "$(git symbolic-ref --short HEAD 2>/dev/null || echo "no branch")";
  public static final String GIT_STATUS = "$(git diff --shortstat HEAD)";
  public static final String BUILD_DATE = "$(date +'%Y-%m-%d')";
  public static final String BUILD_TIME = "$(date +'%r')";
}
EOF