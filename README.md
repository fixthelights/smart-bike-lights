# Project Smart Bike Lights

## Git Crash Course
> This project is opinioned on using command line for our operations. Instructions will be in the command line.

Instructions are adapted from this excellent [tutorial](https://githowto.com).


### 1. Clone this repository
```bash
$ git clone https://github.com/fixthelights/smart-bike-lights.git
```
>Copy everything after the `$`symbol. `$` is a prompt symbol to denote the terminal input. If you see a `#` symbol, that denotes the superuser's prompt (admin prompt) More information [here](https://stackoverflow.com/a/48215530/1117934).

Change directory `cd` into the repository.

```bash
$ cd smart-bike-lights
```

### 2. Making changes to files

For example, lets create a hello-world.py file

```bash
$ touch hello-world.py
```

Check `git status` to see the changes
```bash
$ git status
```

```bash
On branch main
Untracked files:
  (use "git add <file>..." to include in what will be committed)
	hello-world.py

nothing added to commit but untracked files present (use "git add" to track)
```

### 3. Adding changes to staging

Staging is a concept in git that represents an intermediate area where changes are added in preparation for a commit

```bash
$ git add .
```
> Don't forget the dot `.` which denotes `current directory`

Run `git status` to ensure they're staged
```bash
$ git status
```

```bash
On branch main
Changes to be committed:
  (use "git restore --staged <file>..." to unstage)
	new file:   hello-world.py
```

### 4. Finally commit the changes

```bash
$ git commit -m 'Added hello-world.py'
```
> the `-m` flag adds a `commit message` which will show up on github. It should be short and clear, a summary of the commit 

Output: 
```bash
[main 153f6eb] Added hello-world.py
 1 file changed, 1 insertion(+)
 create mode 100644 hello-world.py
```

### 5. Push to the cloud (Github)

```bash
$ git push
```
> This step uploads your commit to Github. Don't forget this step!

> Without uploading, your commit stays on your device only, and others won't be able to see your work.
