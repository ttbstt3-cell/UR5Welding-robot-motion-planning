function report=replay_saved_accp_graph(file,out)
% Read-only replay of the archived graph policy; does NOT revalidate/promote
% raw diagnostic nodes or change today's planner. Mirrors original localBuildGraph.
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真'); addpath(genpath('/home/ttbstt/jixiebi/RRT'));
s=load(file,'result'); r=s.result; scene=r.scene; op=r.options;
queries=HmiExactSceneQueries(scene,op.performanceBinding,true);
report=struct('source',file,'layers',{{}},'edge_tests',0,'connected_edges',0,...
    'collision_rejected_edges',0,'executionReady',false,'robot_motion','NO');
q0Free=queries.isFree(scene.qMat0); reachable=cell(size(r.stateBank));
for i=1:numel(r.stateBank)
    now=r.stateBank{i}; reachable{i}=false(1,numel(now));
    row=struct('layer',i,'candidate_nodes',numel(now),'tested_edges',0,'connected_edges',0,...
        'step_rejected_pairs',0,'pose_step_rejected_pairs',0,'collision_rejected_edges',0);
    if i==1
        reachable{i}(:)=true;
        for j=1:numel(now)
            row.tested_edges=row.tested_edges+1;
            pass=q0Free && edge(scene.qMat0,now(j).q,false,false);
            row.connected_edges=row.connected_edges+pass;
            row.collision_rejected_edges=row.collision_rejected_edges+~pass;
        end
    else
        prev=r.stateBank{i-1};
        for j=1:numel(now)
            for k=1:numel(prev)
                if ~reachable{i-1}(k), continue; end
                roll=abs(rad2deg(atan2(sind(now(j).rollDeg-prev(k).rollDeg),cosd(now(j).rollDeg-prev(k).rollDeg))));
                if roll>op.maxRollStepDeg+1e-9 || abs(now(j).travelDeg-prev(k).travelDeg)>op.maxTravelStepDeg+1e-9
                    row.pose_step_rejected_pairs=row.pose_step_rejected_pairs+1; continue;
                end
                dq=atan2(sin(now(j).q-prev(k).q),cos(now(j).q-prev(k).q));
                if max(abs(rad2deg(dq)))>op.maxJointStepDeg
                    row.step_rejected_pairs=row.step_rejected_pairs+1; continue;
                end
                bridge=(isfield(now(j),'bridgeNode')&&now(j).bridgeNode)||...
                    (isfield(prev(k),'bridgeNode')&&prev(k).bridgeNode);
                row.tested_edges=row.tested_edges+1;
                pass=edge(prev(k).q,now(j).q,bridge,op.lazyCollision);
                row.connected_edges=row.connected_edges+pass;
                row.collision_rejected_edges=row.collision_rejected_edges+~pass;
                reachable{i}(j)=reachable{i}(j)||pass;
            end
        end
    end
    assert(isequal(reachable{i},r.graph.reachable{i}),'Archived reachability differs at layer %d',i);
    row.reachable_states=sum(reachable{i}); report.layers{end+1}=row;
    report.edge_tests=report.edge_tests+row.tested_edges;
    report.connected_edges=report.connected_edges+row.connected_edges;
    report.collision_rejected_edges=report.collision_rejected_edges+row.collision_rejected_edges;
end
assert(report.edge_tests==r.graph.edgeTests && report.connected_edges==r.graph.collisionFreeEdges);
report.archived_graph_counts_exact_match=true;
report.note='Graph policy collision passes do not certify axis/side or precision at unscreened raw endpoints. Final refinement failure is retained.';
f=fopen(out,'w'); c=onCleanup(@()fclose(f)); fwrite(f,jsonencode(report,'PrettyPrint',true),'char');
fprintf('SAVED_GRAPH_REPLAY_PASS tests=%d connected=%d rejected=%d\n',report.edge_tests,report.connected_edges,report.collision_rejected_edges);
    function pass=edge(a,b,isBridge,lazy)
        dq=atan2(sin(b-a),cos(b-a));
        if isBridge, fractions=linspace(0,1,max(2,round(op.cartesianBridgeMaxEdgeSamples)));
        else
            n=max(1,ceil(max(abs(rad2deg(dq)))/op.edgeResolutionDeg));
            if lazy, fractions=(1:n-1)/n; else, fractions=(0:n)/n; end
        end
        pass=true;
        for u=fractions
            if ~queries.isFree(a+u*dq), pass=false; return; end
        end
    end
end
