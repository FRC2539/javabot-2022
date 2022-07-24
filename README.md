# Javabot

The code for FRC team 2539's 2022 robot can be found here.

## Contribution

_If you are new to programming, check out our training repo: [Java Training](https://github.com/FRC2539/java-training)_

### Technical Debt

A robotics season is an intense time, so it is natural that during the process of development we will accumulate technical debt (unclean/haphazard code).

While this is unavoidable, making changes down the line becomes much easier when code is refactored and well-thought-out.

If you have extra time while writing robot code, or extra time at robot, try to consider ways that you can improve your current code so that it can be extended easily by future programmers other than yourself.

Solid code makes development and improvement much easier.

### Adding Features

When adding features or refactoring the code, we recommend that you create a new branch. 

The branch name should reflect the intention of the branch.

Create a new branch with (**replace `<my-branch>` with the name of your branch**):

```bash
git checkout -b <my-branch>
```

### Merging Completed Features

Before merging, there are a few tests that must be verified.

You must test your changes on the actual robot, not just the simulator, before creating a pull request.

Additionally, the github repo will automatically run tests for any commits or pull requests to the master branch.

Make sure that the tests pass locally (they are run during a build), and make sure that spotless check (`./gradlew spotlessCheck`) passes as well.

With that, you are ready to create a pull request, and once everything looks good, a programming team leader will approve the merge.