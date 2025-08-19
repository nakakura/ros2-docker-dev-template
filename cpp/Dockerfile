FROM ubuntu:24.04

ARG UID
ARG GID

# 必要なツールのインストール
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# プロジェクトディレクトリ作成と所有者変更
ARG UID
ARG GID
RUN mkdir -p /workspace/src && chown -R ${UID}:${GID} /workspace
WORKDIR /workspace
# ubuntuユーザーをパスワードなしsudoに設定
RUN apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/*
RUN echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers.d/ubuntu
USER ubuntu
