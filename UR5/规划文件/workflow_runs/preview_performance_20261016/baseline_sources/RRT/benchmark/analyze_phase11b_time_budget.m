function pareto = analyze_phase11b_time_budget()
% analyze_phase11b_time_budget  Recompute Phase 11B Pareto flags.
%
% Success rate is a hard constraint.  A variant with no successful path at a
% budget cannot be Pareto-optimal when another variant succeeds there.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
summary = readtable(fullfile(projectRoot,'results', ...
    'phase11b_time_budget_summary.csv'));
rows = repmat(emptyParetoRow(),0,1);
for mapID = unique(summary.map).'
    for budget = unique(summary.budget_ms).'
        subset = summary(summary.map == mapID & ...
            summary.budget_ms == budget,:);
        for i = 1:height(subset)
            dominated = false;
            iPath = subset.bestPathLengthMedian(i);
            iChecks = subset.collisionChecksMedian(i);
            for j = 1:height(subset)
                if i == j, continue; end
                jPath = subset.bestPathLengthMedian(j);
                jChecks = subset.collisionChecksMedian(j);
                if ~isfinite(jPath) || ~isfinite(jChecks), continue; end
                if ~isfinite(iPath)
                    dominated = subset.successRateByBudget(j) > ...
                        subset.successRateByBudget(i);
                    if dominated, break; end
                    continue;
                end
                successBetterOrEqual = subset.successRateByBudget(j) >= ...
                    subset.successRateByBudget(i);
                qualityBetterOrEqual = jPath <= iPath && jChecks <= iChecks;
                oneStrict = subset.successRateByBudget(j) > ...
                    subset.successRateByBudget(i) || jPath < iPath || ...
                    jChecks < iChecks;
                if successBetterOrEqual && qualityBetterOrEqual && oneStrict
                    dominated = true;
                    break;
                end
            end
            rows(end+1) = struct('variant',subset.variant{i}, ...
                'map',mapID,'mapName',subset.mapName{i}, ...
                'budget_ms',budget,'successRateByBudget', ...
                subset.successRateByBudget(i), ...
                'bestPathLengthMedian',subset.bestPathLengthMedian(i), ...
                'collisionChecksMedian',subset.collisionChecksMedian(i), ...
                'isPareto',~dominated); %#ok<AGROW>
        end
    end
end
pareto = struct2table(rows);
writetable(pareto,fullfile(projectRoot,'results', ...
    'phase11b_time_budget_pareto.csv'));
end

function row = emptyParetoRow()
row = struct('variant','', 'map',NaN, 'mapName','', 'budget_ms',NaN, ...
    'successRateByBudget',NaN, 'bestPathLengthMedian',NaN, ...
    'collisionChecksMedian',NaN, 'isPareto',false);
end
