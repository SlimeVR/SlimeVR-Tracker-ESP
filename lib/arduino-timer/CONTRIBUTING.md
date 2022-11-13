# Contributing

## Coding Style

Try and adhere to the [kernel coding style](https://www.kernel.org/doc/html/latest/process/coding-style.html) with the exception of using 4 spaces instead of 8 for indentation.

## Reporting Bugs

Open an issue. Please include as much code / logging / output / information as
possible to help diagnose your issue.

## Proposing Changes

Open a pull request using a branch based on **master**. Read the [branches
section](#branches) for an overview of the workflow.

If it's a major change, open an issue to gauge interest before putting in too
much effort.

The following subsections inspired by the [git patch guidelines](https://github.com/git/git/blob/master/Documentation/SubmittingPatches).

### Commits

Make separate commits for logically separate changes

The title for the commit should start with the file or subsection it is
changing.

Give an explanation for the change(s) that is detailed enough so that people
can judge if it is good thing to do, without reading the actual patch text to
determine how well the code does what the explanation promises to do.

If your description starts to get too long, that's a sign that you probably
need to split up your commit to finer grained pieces.  That being said, patches
which plainly describe the things that help reviewers check the patch, and
future maintainers understand the code, are the most beautiful patches.
Descriptions that summarize the point in the subject well, and describe the
motivation for the change, the approach taken by the change, and if relevant
how this differs substantially from the prior version, are all good things to
have.

### Basing your branch

In general, always base your work on the oldest branch that your change is
relevant to.

- A bugfix should be based on **master** in general. For a bug that's not yet
  in **master**, find the topic that introduces the regression, and base your
  work on the tip of the topic.

- A new feature should be based on **master** in general. If the new feature
  depends on a topic that is in **test**, but not in **master**, base your
  work on the tip of that topic.

* Corrections and enhancements to a topic not yet in **master** should be based
  on the tip of that topic. If the topic has not been merged to **next**, it's
  alright to add a note to squash minor corrections into the series.

* In the exceptional case that a new feature depends on several topics not in
  **master**, start working on **next** or **test** privately and send out
  patches for discussion. Before the final merge, you may have to wait until
  some of the dependent topics graduate to **master**, and rebase your work.

To find the tip of a topic branch, run `git log --first-parent master..test`
and look for the merge commit. The second parent of this commit is the tip of
the topic branch.

## Branches

Usage inspired by the [git branch workflow](https://github.com/git/git/blob/master/Documentation/howto/maintain-git.txt).

### master

- All new features and changes should be based on this branch
- Is meant to be more stable than any tagged releases, and users are encouraged
  to follow it
- Tagged for releases

### next

- Used to publish changes (both enhancements and fixes) that (1) have
  worthwhile goal, (2) are in a fairly good shape suitable for everyday use,
  (3) but have not yet demonstrated to be regression free.
- Contains all new proposed changes for the next release
- Users are encouraged to use it to test for regressions and bugs before they
  get merged for release
- Periodically rebased on to **master**, usually after a tagged release

### test

- Branch is used to publish other proposed changes that do not yet pass the
  criteria set for **next**.
- Contains all seen proposed feature / enhancement requests
- Most unstable branch
- Regularly rebased on to **master**

### [initials]/[branch-name]

- Name based on the author's initials and topic of the branch. For example
  *mc/fix-bug-in-foo*
- Based on **master**, unless it requires code from another feature, in which
  case base it on that feature branch
- Contains a proposed feature / enhancement / bug fix wanting to be merged
- It will first get merged into **test**, then **next**, and finally **master**
  when it's ready

## Versioning

Version numbers follow [Semantic Versioning](https://semver.org/).
