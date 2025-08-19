FROM ros:jazzy-ros-base

ARG UID
ARG GID

# 必要なツールのインストール
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# ワークスペース作成と所有者変更
RUN mkdir -p /ros2_ws/src && chown -R ${UID}:${GID} /ros2_ws
# ubuntuユーザーをパスワードなしsudoに設定
# 運用時はこの設定を削除し、sudoerファイルを適切に管理することを推奨
RUN echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers.d/ubuntu
WORKDIR /ros2_ws
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/ubuntu/.bashrc
# colcon build --symlink-install（srcが空でもエラーにならないようtrueで吸収）
RUN colcon build --symlink-install || true
# build, install, logディレクトリの所有者をubuntuユーザーに変更
RUN chown -R ${UID}:${GID} /ros2_ws/build /ros2_ws/install /ros2_ws/log || true
# install/setup.bashが存在する場合のみ自動でsource
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /home/ubuntu/.bashrc
