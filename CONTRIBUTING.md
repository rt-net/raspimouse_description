# Contribution Guide

本リポジトリへのコントリビュート方法について記載しています。

## Issues

リポジトリの品質向上にご協力頂きありがとうございます。

Issueの作成を簡単にするテンプレートを用意しているので活用してください。

## Pull Requests

Pull Requestの作成ありがとうございます。
提出したPull Request（PR）には次のルールが適用されます。

- PRの内容には本リポジトリのライセンス（[LICENSE](./LICENSE)と[README.md](./README.md)に記載されています）が適用されます
- PRは`rt-net`のメンバーによるレビューを経てからマージされます
  - すべてのPRがマージされるわけではなく、希望に添えない場合もありますのでご容赦ください
- リポジトリにテストが設定されている場合はできるだけテストを通してください
  - 何かしらの理由（テストに間違いがある場合など）でテストを通さずPRを出す場合はその旨をPRに記載してください
- マージする際にはPR内の全コミットが1つのコミットに`squash`されます
  - [コミットをスカッシュしてマージする | GitHub Docs](https://docs.github.com/ja/pull-requests/collaborating-with-pull-requests/incorporating-changes-from-a-pull-request/about-pull-request-merges#squash-and-merge-your-commits)
- 1つのPRでリクエストする変更はできるだけシンプルにしてください
  - 異なる内容の変更を含む場合はPRを分割してください
      - 例えば、複数の機能追加したり、機能追加とリファクタリングを同時にする場合はそれぞれ別々のPRとしてください
  - squashマージしても履歴を辿りやすくするためです

## LICENSE

Any contribution that you make to this repository will
be under the MIT license, as dictated by that
[license](https://opensource.org/licenses/MIT).
