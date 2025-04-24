classdef PriorityQueue < handle
    properties
        elements
    end

    methods
        function obj = PriorityQueue()
            obj.elements = {};
        end

        function insert(obj, node, key)
            obj.elements{end+1} = {node, key};
            keys = cellfun(@(x) x{2}, obj.elements, 'UniformOutput', false);
            keys = cell2mat(keys);
            [~, order] = sortrows(keys);
            obj.elements = obj.elements(order);
        end

        function [node, key] = top(obj)
            node = obj.elements{1}{1};
            key = obj.elements{1}{2};
        end

        function pop(obj)
            obj.elements(1) = [];
        end

        function isEmpty = isEmptyQueue(obj)
            isEmpty = isempty(obj.elements);
        end
    end
end