classdef HmiExactSceneQueries < handle
    % Exact request-local memoization over an immutable scene snapshot.
    % No tolerance, angle wrapping, cross-request or on-disk validity cache.
    properties (Access=private)
        Scene
        EnvironmentResults
        SelfResults
        ForwardResults
        Enabled
        Stats
        MaxEntries = 100000
        Binding
    end
    methods
        function obj=HmiExactSceneQueries(scene,binding,enabled)
            if nargin<3, enabled=true; end
            obj.Scene=scene; obj.Binding=binding; obj.Enabled=logical(enabled);
            obj.EnvironmentResults=containers.Map('KeyType','char','ValueType','any');
            obj.SelfResults=containers.Map('KeyType','char','ValueType','any');
            obj.ForwardResults=containers.Map('KeyType','char','ValueType','any');
            obj.Stats=struct('environment_queries',0,'environment_hits',0,'environment_native_sec',0,...
                'self_queries',0,'self_hits',0,'self_native_sec',0,...
                'fk_queries',0,'fk_hits',0,'fk_native_sec',0,...
                'ik_calls',0,'ik_native_sec',0);
        end
        function [hit,distance]=environment(obj,q)
            obj.Stats.environment_queries=obj.Stats.environment_queries+1;
            key=obj.key(q);
            if obj.Enabled && isKey(obj.EnvironmentResults,key)
                r=obj.EnvironmentResults(key); obj.Stats.environment_hits=obj.Stats.environment_hits+1;
            else
                clock=tic;
                [r.hit,r.distance]=check_scene_environment_collision(obj.Scene,q);
                obj.Stats.environment_native_sec=obj.Stats.environment_native_sec+toc(clock);
                if obj.Enabled && obj.EnvironmentResults.Count<obj.MaxEntries, obj.EnvironmentResults(key)=r; end
            end
            hit=r.hit; distance=r.distance;
        end
        function [hit,distance]=self(obj,q)
            obj.Stats.self_queries=obj.Stats.self_queries+1; key=obj.key(q);
            if obj.Enabled && isKey(obj.SelfResults,key)
                r=obj.SelfResults(key); obj.Stats.self_hits=obj.Stats.self_hits+1;
            else
                clock=tic;
                [r.hit,r.distance]=checkCollision(obj.Scene.robot,q,'IgnoreSelfCollision','off',...
                    'SkippedSelfCollisions',obj.Scene.selfCollisionSkipPairs,'Exhaustive','on');
                obj.Stats.self_native_sec=obj.Stats.self_native_sec+toc(clock);
                if obj.Enabled && obj.SelfResults.Count<obj.MaxEntries, obj.SelfResults(key)=r; end
            end
            hit=r.hit; distance=r.distance;
        end
        function free=isFree(obj,q)
            env=obj.environment(q);
            if any(env(:)), free=false; return; end
            own=obj.self(q); free=~any(own(:));
        end
        function T=forward(obj,q,body)
            obj.Stats.fk_queries=obj.Stats.fk_queries+1;
            key=[char(body) ':' obj.key(q)];
            if obj.Enabled && isKey(obj.ForwardResults,key)
                T=obj.ForwardResults(key); obj.Stats.fk_hits=obj.Stats.fk_hits+1; return;
            end
            clock=tic; T=getTransform(obj.Scene.robot,q,char(body));
            obj.Stats.fk_native_sec=obj.Stats.fk_native_sec+toc(clock);
            if obj.Enabled && obj.ForwardResults.Count<obj.MaxEntries, obj.ForwardResults(key)=T; end
        end
        function s=summary(obj)
            s=obj.Stats; s.enabled=obj.Enabled; s.binding=obj.Binding;
            s.environment_native_queries=s.environment_queries-s.environment_hits;
            s.self_native_queries=s.self_queries-s.self_hits;
            s.fk_native_queries=s.fk_queries-s.fk_hits;
            s.scope='One immutable input scene in one function/request; discarded on return';
            s.key='IEEE754 num2hex of all six raw joint values; no rounding or wrapping';
        end
        function recordIK(obj,elapsed)
            obj.Stats.ik_calls=obj.Stats.ik_calls+1;
            obj.Stats.ik_native_sec=obj.Stats.ik_native_sec+elapsed;
        end
    end
    methods (Static,Access=private)
        function value=key(q)
            value=reshape(num2hex(reshape(double(q),[],1)).',1,[]);
        end
    end
end
