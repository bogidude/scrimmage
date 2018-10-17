/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/interaction/GraphInteraction/GraphUtils.h>

#include <scrimmage/plugin_manager/Plugin.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/msgs/Graph.pb.h>

#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <GeographicLib/LocalCartesian.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/transform.hpp>


namespace ba = boost::adaptors;
namespace br = boost::range;
namespace sp = scrimmage_proto;

namespace scrimmage {
namespace interaction {

GraphInteraction::Graph read_graph(std::string filename) {
    std::ifstream graph_file(filename);

    GraphInteraction::Graph g_;
    boost::dynamic_properties dp(boost::ignore_other_properties);
    dp.property("osmid", boost::get(&GraphInteraction::VertexProperties::osmid, g_));
    dp.property("x", boost::get(&GraphInteraction::VertexProperties::x, g_));
    dp.property("y", boost::get(&GraphInteraction::VertexProperties::y, g_));

    dp.property("geometry", boost::get(&GraphInteraction::EdgeProperties::geometry, g_));
    dp.property("length", boost::get(&GraphInteraction::EdgeProperties::length, g_));
    dp.property("osmid", boost::get(&GraphInteraction::EdgeProperties::osmid, g_));
    dp.property("name", boost::get(&GraphInteraction::EdgeProperties::name, g_));

    if (graph_file.is_open()) {
        boost::read_graphml(graph_file, g_, dp);
        std::cout << "read graph file" << std::endl;
        graph_file.close();
    }
    return g_;
}

scrimmage_msgs::Graph boost2protoGraph(GraphInteraction::Graph boost_g, EntityPtr parent_) {
    std::map<int64_t, Eigen::Vector3d> nodes;
    std::map<int64_t, int64_t> boost_vert_to_osmid;
    int id_ = 1;
    auto graph_msg = std::make_shared<scrimmage::Message<scrimmage_msgs::Graph>>();
    graph_msg->data.set_id(id_);
    for (auto vs = boost::vertices(boost_g); vs.first != vs.second; ++vs.first) {
        double longitude = boost_g[*vs.first].x;
        double latitude = boost_g[*vs.first].y;
        double this_osmid = boost_g[*vs.first].osmid;
        boost_vert_to_osmid[*vs.first] = this_osmid;

        double x, y, z;
        parent_->projection()->Forward(latitude, longitude, 0.0, x, y, z);
        nodes[this_osmid] = Eigen::Vector3d(x, y, z);
        auto node_ptr = graph_msg->data.add_nodes();
        node_ptr->set_id(this_osmid);
        scrimmage::set(node_ptr->mutable_point(), nodes[this_osmid]);
    }

    for (auto es = boost::edges(boost_g); es.first != es.second; ++es.first) {
        int64_t id_start = boost_vert_to_osmid[boost::source(*es.first, boost_g)];
        int64_t id_end = boost_vert_to_osmid[boost::target(*es.first, boost_g)];

        auto edge_ptr = graph_msg->data.add_edges();
        edge_ptr->set_start_node_id(id_start);
        edge_ptr->set_end_node_id(id_end);
        edge_ptr->set_weight(boost_g[*es.first].length);
        edge_ptr->set_label(boost_g[*es.first].name);
    }
}

void draw_graph(
        scrimmage_msgs::Graph &graph,
        const std::unordered_map<uint64_t, scrimmage_proto::Vector3d> &node_idx_to_pos,
        DrawNodeLabels draw_node_labels,
        std::shared_ptr<Plugin> plugin) {

    const std::vector<int> black {0, 0, 0};
    const std::vector<int> white {255, 255, 255};
    const std::vector<int> blue {0, 0, 255};

    for (auto &e : graph.edges()) {
        auto edge_shape = std::make_shared<sp::Shape>();
        set(edge_shape->mutable_color(), black);
        edge_shape->set_opacity(1.0);
        edge_shape->set_persistent(true);

        auto &p1 = node_idx_to_pos.at(e.start_node_id());
        auto &p2 = node_idx_to_pos.at(e.end_node_id());

        set(edge_shape->mutable_line()->mutable_start(), p1.x(), p1.y(), p1.z());
        set(edge_shape->mutable_line()->mutable_end(), p2.x(), p2.y(), p2.z());
        plugin->draw_shape(edge_shape);
    }

    auto node_shape = std::make_shared<sp::Shape>();
    node_shape->set_persistent(true);
    auto to_pt = [&](auto &n) {return n.point();};
    for (const auto &pt : graph.nodes() | ba::transformed(to_pt)) {
        set(node_shape->mutable_pointcloud()->add_point(), pt.x(), pt.y(), pt.z());
        set(node_shape->mutable_pointcloud()->add_color(), blue);
    }
    node_shape->mutable_pointcloud()->set_size(6);
    plugin->draw_shape(node_shape);

    for (const auto &node : graph.nodes()) {
        auto text_shape = std::make_shared<sp::Shape>();
        text_shape->set_persistent(true);
        text_shape->set_opacity(1.0);
        set(text_shape->mutable_color(), white);

        if (draw_node_labels == DrawNodeLabels::YES) {
            Eigen::Vector3d pos(node.point().x(), node.point().y(), node.point().z());
            pos(0) += 5;
            set(text_shape->mutable_text()->mutable_center(), pos);
            text_shape->mutable_text()->set_scale(20);

            text_shape->mutable_text()->set_text(std::to_string(node.id()));
            plugin->draw_shape(text_shape);
        }
    }
}

std::unordered_map<uint64_t, scrimmage_proto::Vector3d> nodes_idxs_to_pos_map(
        const scrimmage_msgs::Graph &graph) {

    auto to_pos = [&](auto &node) {return std::make_pair(node.id(), node.point());};
    std::unordered_map<uint64_t, scrimmage_proto::Vector3d> out;
    br::transform(graph.nodes(), std::inserter(out, out.begin()), to_pos);
    return out;
}
} // namespace interaction
} // namespace scrimmage
