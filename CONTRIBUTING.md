# Contributing

Thanks for helping with SlimeVR! This document explains what to know when
contributing.

We are most active on [Discord](https://discord.gg/SlimeVR), come chat with us if you
have any questions or want to hang out! 😎

## Setting up the development environment

We use PlatformIO as our build tooling and rely on Git for version control. There are
instructions to set all of this up [here](https://docs.slimevr.dev/firmware/setup-and-install.html).

## Using third-party code

Often you will want to use or reference third-party code. You should not just
copy-paste code from random libraries! There are several reasons for this:

- It is not clear when/where the code originated from.
- The original code's license needs to be kept intact.
- It removes the ability to update our code when the original updates.
- It makes it harder for us to maintain the code, because we don't know what has changed
  from the original!

Instead, we will want to do the following:
1. Check that the dependency has a permissive license.
1. (Optional) Fork the dependency if it needs any modification.
1. Add the dependency via PlatformIO.

### Check the license

The SlimeVR firmware is under the MIT License and also is intended to be permissively
licensed. This means that we will not accept contributions that include code from any
viral software licenses. If you do not know what that means, here is a breakdown of some
common software licenses:

> Fine to use:
> * MIT
> * Apache 2.0
> * BSD
>
> Will not be accepted:
> * GPLv3 or v2
> * LGPL
> * No license

This includes the license of any code that your dependency depends on! If you are ever
unsure, feel free to ask us in our Discord.

### (Optional) Fork the dependency

When you want to use the depdendency, you should ask yourself: "Can I simply call the
functions/classes in the dependency from *my* code, or do I actually need to modify the
source code of the original files"? If the answer is "I need to modify the original
files", then you will need to fork the dependency so that your modifications are clear.

> **Note**
> If you have never forked on GitHub before, a fork is basically a copy of a library that
> maintains the original's history. This means that any changes you commit in your fork
> will show up as changes against the original. We can even update these changes as the
> original changes!

To create a fork in GitHub, simply go to the original library and click this button:
![](https://docs.github.com/assets/cb-28613/images/help/repository/fork_button.png)
Now clone your repo to your computer. From this point forward, any changes that you
make while working on your fork will have their Git history preserved and linked to the
original when you commit those changes.

We will likely ask you to transfer ownership of the fork to the SlimeVR organization if
your PR is going to be merged.


### Adding the dependency

PlatformIO provides us a helpful way to do dependency management. 
Here we provide the basic workflow you can follow, but you can refer to the 
[PlatformIO documentation on dependecies][dep docs] for more details.

[dep docs]: https://docs.platformio.org/en/latest/librarymanager/dependencies.html

PlatformIO gives us a helpful way to do dependency management, which you can read an
overview of [here](https://docs.platformio.org/en/latest/librarymanager/dependencies.html).
But to save you time, we have outlined the basic workflow for you.

There is a field called `lib_deps` in the `platformio.ini` file that describes the list
of dependencies in the code. These are third-party libraries located outside the main
firmware git repo. You want to add your dependency to this list. The full list of
options for this field is outlined [here], but for simplicity, there are a few common
ways of giving dependencies:

[here]: https://docs.platformio.org/en/latest/core/userguide/pkg/cmd_install.html#cmd-pkg-install-specifications

```ini
[env]
lib_deps=
  bblanchon/ArduinoJson@6.9.12
  https://github.com/Some/Dependency.git#v1.2.3
  symlink://C:/path/to/the/library
```

In order, these dependencies do the following:
1. Adds a dependency on version `6.9.12` of the
  [ArduinoJson](https://registry.platformio.org/libraries/bblanchon/ArduinoJson)
  library by `bblanchon`. This library was uploaded to the PlatformIO
  [package registry](https://registry.platformio.org/search) which contains many popular
  libraries intended for use with PlatformIO.
1. Adds a dependency to a Git repo located at `https://github.com/Some/Dependency` and
  uses the commit tagged as `v1.2.3`. This could also have been a branch name, or a
  commit hash.
1. Links to a folder locally on your computer with the path `C:/path/to/the/library`.
  This is useful when testing your code locally where you want to make changes in your
  fork (which is checked out at that path), but you don't want to have to constantly
  commit and push to your fork just to get the changes usable by the firmware.

> **Note**
> If you have forked the dependency instead of using it as-is, we recommend using the
> `symlink://` approach while developing locally, and then using the Git approach when
> submitting your code for a pull request. This is because while developing locally,
> you will want to test out frequent changes and won't want to have to push constantly
> to Git and have PlatformIO constantly retrieve those changes. However, for anyone
> else to be able to use your code, it needs to be in a Git repository rather than a symlink.
>
> Once you are done doing local testing and are ready to submit a PR, you can push
> whatever changes you made to the forked dependency to its repository, and switch the
> firmware's `lib_deps` back to the Git url.
>
> Dont forget that your fork will need to be pushed and publicly viewable on GitHub for
> us to be able to use and review it!
