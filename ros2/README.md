# ROS 2 Jazzy rclcpp Docker Template

このテンプレートは、ROS 2 Jazzy (rclcpp/C++) 開発環境をDocker Composeで簡単に構築できるプロジェクト雛形です。

## 特長
- 公式ROS 2 Jazzyイメージをベース
- ホストユーザーのUID/GIDに合わせたパーミッション設定
- パスワードなしsudo
- .env自動生成スクリプト付き
- サンプルパッケージ追加も容易

## セットアップ手順
1. 必要ツールのインストール
   - Docker
   - Docker Compose

2. このリポジトリをクローン

3. `.env`ファイルの生成(自動)
   ```bash
   ./launch.sh
   ```
   ※初回はイメージビルドとコンテナ起動まで自動実行されます

4. コンテナに入る
   ```bash
   docker compose exec ros2 bash
   ```

5. ROS 2コマンドやcolcon buildがそのまま使えます

## srcディレクトリ
- `src/`配下にrclcppパッケージを追加してください
- 例: 
   ```bash
   cd src
   ros2 pkg create --build-type ament_cmake my_cpp_pkg --dependencies rclcpp
   cd ..
   colcon build --symlink-install
   ```

## 注意事項
- 運用ではなく開発目的のため、コンテナ内での `sudo` はパスワードなしで実行できます(セキュリティに注意)
- Dockerfileやcomposeファイルのバージョンは適宜調整してください

## ライセンス
このプロジェクトはMIT Licenseで公開されています。
詳細はLICENSEファイルを参照してください。
