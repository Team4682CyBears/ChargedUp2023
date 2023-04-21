
$blueAllianceApiBase = "https://www.thebluealliance.com/api/v3"
$authKey = "F9XCNPPfm6yEHD3s4hqTrzBEbZnHbnSq37xXmRCDKv3Si2l61gLISHOvt7CMNTaJ"

$headers = @{
    'X-TBA-Auth-Key'=$authKey
    Content='application/json'
}

$eventsApi = $blueAllianceApiBase + "/events/2023"

$eventResults = Invoke-RestMethod -Method Get -Uri $eventsApi -Headers $headers

ForEach($nextEvent in $eventResults)
{
    if($nextEvent.city -eq "Houston" -and $nextEvent.event_type -eq 3)
    {
#       Write-Host "**********************************************"
#       Write-Host $nextEvent.name

       $matchesApi = $blueAllianceApiBase + "/event/" + $nextEvent.key + "/matches"

       $matches = Invoke-RestMethod -Method Get -Uri $matchesApi -Headers $headers
       ForEach($nextMatch in $matches)
       {
           $totalScore = $nextMatch.alliances.blue.score + $nextMatch.alliances.red.score
           if($totalScore -gt -1)
           {
                $lineToLog =  "" + $nextMatch.match_number + "_" + $nextEvent.key + ", " + $nextMatch.alliances.blue.score + ", " + $nextMatch.alliances.red.score + ", " + $totalScore
                Write-Output $lineToLog
           }
       }

    }
}

Write-Host ""
