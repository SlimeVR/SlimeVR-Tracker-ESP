# Contributing

Thanks for helping with SlimeVR! This document explains what to know when
contributing.

We are most active on [discord](https://discord.gg/SlimeVR), come chat with us if you
have any questions or want to hang out! 😎


## Setting up PlatformIO

We use platformio as our build tooling. To set this up, we recommend using the
[VSCode](https://code.visualstudio.com/) editor, and installing the platformio
extension:
![](https://docs.platformio.org/en/latest//_images/platformio-ide-vscode-pkg-installer.png)

There is also a [command line version](https://docs.platformio.org/en/latest//core/index.html#piocore)
of platformio if you prefer that, that you can use instead.


## Using third-party code

Often you will want to use or reference third party code. You should not just
copy-paste code from random libraries! There are several reasons for this:

- It is not clear when/where the code originated from.
- The original code's license needs to be kept intact.
- It removes the ability to update our code when the original updates.
- It makes it harder for us to maintain the code, because we don't what has changed
  from the original!

To bring a new library into the firmware, we instead treat the modified library as a
"fork" of the original. We then add the forked library as a dependency of the firmware,
via platformio which is our build tool.


### Forking

> **Note**
> If you have never forked in git before, a fork is basically a copy of a library that
> maintains the original's history. This means that any changes you commit in your fork
> will show up as changes against the original. We can even update these changes as the
> original changes!

To create a fork in github, simply go to the original library and click this button:
![](https://docs.github.com/assets/cb-28613/images/help/repository/fork_button.png)
Now clone your repo to your computer. From this point forward, any changes that you
make while working on your fork will have their git history preserved and linked to the
original.

### Adding the fork as a dependency

Platformio gives us a helpful way to do dependency management, which you can read an
overview of [here](https://docs.platformio.org/en/latest/librarymanager/dependencies.html).
But to save you time, we have outlined the basic workflow for you.

There is a field called `lib_deps` of the `platformio.ini` file that describes the list
of dependencies in the code. These are third-party libraries not located in the main
firmware git repo. You want to add your fork to this list. The full list of options for
this field is outlined [here](https://docs.platformio.org/en/latest/core/userguide/pkg/cmd_install.html#cmd-pkg-install-specifications)
but for simplicity, there are a few common ways of giving dependencies:

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
  library by `bblanchon`. This library was uploaded to the platformio
  [package registry](https://registry.platformio.org/search) which contains many popular
  libraries intended for use with platformio.
2. Adds a dependency to a git repo located at `https://github.com/Some/Dependency` and
  uses the commit tagged as `v1.2.3`. This could also have been a branch name, or a
  commit hash.
3. Links to a folder locally on your computer with the path `C:/path/to/the/library`.
  This is useful when testing your code locally where you want to make changes in your
  fork (which is checked out at that path), but you don't want to have to constantly
  commit and push to your fork just to get the changes usable by the firmware.

We recommend using the `symlink://` approach while developing locally, and then using
the git approach when submitting your code for a pull request. Dont forget that your
fork will need to be pushed and publicly viewable on github for us to be able to use
and review it!

We will likely ask you to transfer ownership of the fork to the SlimeVR organization if
your PR is going to be merged.


### Licenses of third party code.

The firmware is under the MIT License and also is intended to be permissively licensed.
This means that we will not accept contributions that include code from any viral
software licenses. If you do not know what that means, here is a breakdown of some
common software licenses:

> Fine for use:
> * MIT
> * Apache 2.0
> * BSD
>
> Will not be accepted:
> * GPLv3 or v2
> * LGPL
> * No license

This includes the license of any code that your dependency depends on! If you are ever
unsure, feel free to ask us on discord.
