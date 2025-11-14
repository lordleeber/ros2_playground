**Steps:**

1.  **Update Version & Changelog:**
    Before creating a new release, ensure the `<version>` tag in your `package.xml` is updated to the new version number (e.g., `1.0.3`).

2.  **Commit and Tag:**
    Commit the version change and any other code changes. Then, create a Git tag that exactly matches the version in `package.xml` and push it to the remote.
    ```bash
    # Example for version 1.0.3
    git branch -D feat/deb-package
    git push origin --delete feat/deb-package
    git checkout -b feat/deb-package
    git push origin feat/deb-package
    git add .
    git commit -m "Release version 1.0.3"
    git push origin feat/deb-package
    git tag 1.0.3
    git push origin 1.0.3
    ```

3.  **Update the Release Repository with Bloom:**
    `bloom` is used to generate and update a separate *release repository* which contains the packaging instructions. From your source code directory (`skai-edge`), run:
    ```bash
    bloom-release --rosdistro humble --track humble skai_edge_perception
    ```
    - For the very first release, `bloom` will ask a series of setup questions.
    - For subsequent releases, `bloom` will detect the new tag and ask to proceed. Follow the prompts, and at the end, ***DECLINE to open a pull request for `ros/rosdistro`.***

4.  **Build the `.deb` Package:**
    Once `bloom` has pushed the packaging instructions for the new version to the release repository (e.g., `skai-edge-release`), you can build the `.deb` file.
    ```bash
    # 1. Clone or navigate to your release repository
    # cd /path/to/skai-edge-release

    # 2. Ensure it's up-to-date with the remote
    git fetch origin
    git checkout master
    git reset --hard origin/master

    # 3. Checkout the packaging branch and sync it
    git checkout debian/humble/jammy/skai_edge_perception
    git reset --hard origin/debian/humble/jammy/skai_edge_perception

    # 4. Clean the workspace and build
    git clean -fdx
    gbp buildpackage -us -uc --git-ignore-branch
    ```
    The final `.deb` package will be created in the parent directory.
    