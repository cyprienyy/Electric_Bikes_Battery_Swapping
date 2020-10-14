"""
initialize infFactor
initialize ban list
initialize start neighborhood
currentIncumbent := solution from tour construction
for maxIter do
    for each neighborhood do
        for maxNbIter do
            N := neighbor of currentIncumbent
            currentIncumbent := min(n){cost(n) | ∀n ∈ N : cost(n) ∈/ ban list}
            update infFactor
            update ban list
            if reached maxNbNonImpIter then
                change neighborhood
                reset currentIncumbent to best feasible solution found so far
                reset infFactor
                break
            end if
        end for
        change neighborhood
        reset currentIncumbent to best feasible solution found so far
        reset infFactor
    end for
    if reached maxNonImpIter then
        break
    end if
end for
"""
