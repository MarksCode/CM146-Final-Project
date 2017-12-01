## CM146 Final Project - A TagPro Bot

#### Instructions to use the script:

1. Install [Tampermonkey Chrome Extension](https://chrome.google.com/webstore/detail/tampermonkey/dhdgffkkebhmkfjojejmpbldmpobfkfo?hl=en).

2. In the extension, create a new script, place contents of TP_Bot.js inside, and save.

3. Go to [TagPro Maptest Server](http://maptest3.newcompte.fr/) and make a new group.

4. Join the red team and launch the game, these are recommended settings:
    1. Map: Open Field Masters (this will eventually be the labyrinth we'll create)
    2. Waiting Time: 1 Second (reduces countdown before game start)

5. Bot behavior should take over at this point!

6. If you want to add another player to the game (i.e. the human player), open another browser (Firefox, Safari, etc.), join group using group link, and join the game. Note that human player's name should be set to "Player 1" (Login then set display name in profile).


#### Instructions for making/testing labyrinth:

###### Making:

1. Download Labyrinth.png and Labyrinth.json.

2. Go to [TagPro Map Editor](http://unfortunate-maps.jukejuice.com/editor).

3. Press the Import/Export Map button and upload the files.

4. Make some cool stuff! Be sure to save once done.

###### Testing:

1. Download and upload .png, .json files to [TagPro Map Hosting site](http://unfortunate-maps.jukejuice.com/).

2. In step 4 of instructions on using the script, under map use following setup:
    ```
      Under Map:
        In text box, put the id from the last part of your uploaded map's url
        Select "http://unfortunate-maps.jukejuice.com/" in bottom dropdown
        Select "Map from some website id" in top dropdown
    ```

3. Launch the game as usual.

