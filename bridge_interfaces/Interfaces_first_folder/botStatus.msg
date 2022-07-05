#unique identifier for a specific bot
string botID

#one of the following: 'load', 'transport', 'charge', 'move', 'defect'
string botStatus

#textual description of the bot, does not have to be unique, might be used for Identification or types of bots
string botName

#load status of powers supply in % fro 0 .. 100
uint8 botAkkuStatus

#unique identifier for specific tool attached to the bot
string botAttachedToolID

#textual description of the tool attached to the bot, does not have to be unique, might be used for Identification or types of tools
string botAttachedToolName

#unique identifier of actual job the bot is integrated to
string botActualJobID

#unique identifier of nexct planned job for the bot
string botNextPlannedJobID

#actual role of the bot for actual job, one of the following: 'master', 'slave'
string botActualRole

#actual name of master for bot in case bot is slave to a job, otherwise other content
string botActualteam_leader

#tbd 3 or 6 DOF (3 should be sufficient)
float32[] botPositionOrientation
