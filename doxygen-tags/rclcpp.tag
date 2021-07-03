<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>allocator_common.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/allocator/</path>
    <filename>allocator__common_8hpp.html</filename>
    <includes id="allocator__deleter_8hpp" name="allocator_deleter.hpp" local="yes" imported="no">rclcpp/allocator/allocator_deleter.hpp</includes>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::allocator</namespace>
    <member kind="typedef">
      <type>typename std::allocator_traits&lt; Alloc &gt;::template rebind_traits&lt; T &gt;</type>
      <name>AllocRebind</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>a89c84a2945dc1cea1d6dfd4fa72a9dcd</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void *</type>
      <name>retyped_allocate</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>a9e8e54de535dc24500c6b0bd40f66b45</anchor>
      <arglist>(size_t size, void *untyped_allocator)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>retyped_deallocate</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>aaefceff62e574693508452a7a125d9c9</anchor>
      <arglist>(void *untyped_pointer, void *untyped_allocator)</arglist>
    </member>
    <member kind="function">
      <type>void *</type>
      <name>retyped_reallocate</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>a670a5abe8a315e3fdea5e2a1119ad72f</anchor>
      <arglist>(void *untyped_pointer, size_t size, void *untyped_allocator)</arglist>
    </member>
    <member kind="function">
      <type>rcl_allocator_t</type>
      <name>get_rcl_allocator</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>a818a205bf62909c16a2f6439eb4c6a1f</anchor>
      <arglist>(Alloc &amp;allocator)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>allocator_deleter.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/allocator/</path>
    <filename>allocator__deleter_8hpp.html</filename>
    <class kind="class">rclcpp::allocator::AllocatorDeleter</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::allocator</namespace>
    <member kind="typedef">
      <type>typename std::conditional&lt; std::is_same&lt; typename std::allocator_traits&lt; Alloc &gt;::template rebind_alloc&lt; T &gt;, typename std::allocator&lt; void &gt;::template rebind&lt; T &gt;::other &gt;::value, std::default_delete&lt; T &gt;, AllocatorDeleter&lt; Alloc &gt; &gt;::type</type>
      <name>Deleter</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>a885f50f2cbbab914f65ef687a0edd61b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_allocator_for_deleter</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>af9fd8ff79de9473a17da7526381be91b</anchor>
      <arglist>(D *deleter, Alloc *alloc)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_allocator_for_deleter</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>a4aaee2843f9b3aff54aa8ec555653431</anchor>
      <arglist>(std::default_delete&lt; T &gt; *deleter, std::allocator&lt; U &gt; *alloc)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_allocator_for_deleter</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>aaaecd19630abe2213cdcf4a3e1a65790</anchor>
      <arglist>(AllocatorDeleter&lt; T &gt; *deleter, Alloc *alloc)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>any_executable.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>any__executable_8hpp.html</filename>
    <includes id="callback__group_8hpp" name="callback_group.hpp" local="yes" imported="no">rclcpp/callback_group.hpp</includes>
    <includes id="client_8hpp" name="client.hpp" local="yes" imported="no">rclcpp/client.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="service_8hpp" name="service.hpp" local="yes" imported="no">rclcpp/service.hpp</includes>
    <includes id="subscription_8hpp" name="subscription.hpp" local="yes" imported="no">rclcpp/subscription.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="struct">rclcpp::AnyExecutable</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::executor</namespace>
    <member kind="typedef">
      <type>AnyExecutable</type>
      <name>AnyExecutable</name>
      <anchorfile>namespacerclcpp_1_1executor.html</anchorfile>
      <anchor>ab79f55088380403dbd5e042248e74ce8</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>any_service_callback.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>any__service__callback_8hpp.html</filename>
    <includes id="function__traits_8hpp" name="function_traits.hpp" local="yes" imported="no">rclcpp/function_traits.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::AnyServiceCallback</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>any_subscription_callback.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>any__subscription__callback_8hpp.html</filename>
    <includes id="allocator__common_8hpp" name="allocator_common.hpp" local="yes" imported="no">rclcpp/allocator/allocator_common.hpp</includes>
    <includes id="function__traits_8hpp" name="function_traits.hpp" local="yes" imported="no">rclcpp/function_traits.hpp</includes>
    <includes id="message__info_8hpp" name="message_info.hpp" local="yes" imported="no">rclcpp/message_info.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::AnySubscriptionCallback</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>callback_group.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>callback__group_8hpp.html</filename>
    <includes id="client_8hpp" name="client.hpp" local="yes" imported="no">rclcpp/client.hpp</includes>
    <includes id="publisher__base_8hpp" name="publisher_base.hpp" local="yes" imported="no">rclcpp/publisher_base.hpp</includes>
    <includes id="service_8hpp" name="service.hpp" local="yes" imported="no">rclcpp/service.hpp</includes>
    <includes id="subscription__base_8hpp" name="subscription_base.hpp" local="yes" imported="no">rclcpp/subscription_base.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::CallbackGroup</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
    <namespace>rclcpp::callback_group</namespace>
    <member kind="typedef">
      <type>CallbackGroupType</type>
      <name>CallbackGroupType</name>
      <anchorfile>namespacerclcpp_1_1callback__group.html</anchorfile>
      <anchor>a57d7989f0fc39e917df95b929fef8d0c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>CallbackGroup</type>
      <name>CallbackGroup</name>
      <anchorfile>namespacerclcpp_1_1callback__group.html</anchorfile>
      <anchor>a88b076618efae1cda12782eba9513d63</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>CallbackGroupType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a0027d5804ef28f0b6fea8eea4195c44a</anchor>
      <arglist></arglist>
      <enumvalue file="namespacerclcpp.html" anchor="a0027d5804ef28f0b6fea8eea4195c44aa0a658d9024420a1c2f737e8881406f7d">MutuallyExclusive</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a0027d5804ef28f0b6fea8eea4195c44aa49816c033d9f3ad7e81fdb953fe3251f">Reentrant</enumvalue>
    </member>
  </compound>
  <compound kind="file">
    <name>client.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>client_8hpp.html</filename>
    <includes id="exceptions_8hpp" name="exceptions.hpp" local="yes" imported="no">rclcpp/exceptions.hpp</includes>
    <includes id="function__traits_8hpp" name="function_traits.hpp" local="yes" imported="no">rclcpp/function_traits.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__graph__interface_8hpp" name="node_graph_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_graph_interface.hpp</includes>
    <includes id="type__support__decl_8hpp" name="type_support_decl.hpp" local="yes" imported="no">rclcpp/type_support_decl.hpp</includes>
    <includes id="utilities_8hpp" name="utilities.hpp" local="yes" imported="no">rclcpp/utilities.hpp</includes>
    <includes id="expand__topic__or__service__name_8hpp" name="expand_topic_or_service_name.hpp" local="yes" imported="no">rclcpp/expand_topic_or_service_name.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::ClientBase</class>
    <class kind="class">rclcpp::Client</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>clock.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>clock_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="time_8hpp" name="time.hpp" local="yes" imported="no">rclcpp/time.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::JumpHandler</class>
    <class kind="class">rclcpp::Clock</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>context.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>context_8hpp.html</filename>
    <includes id="init__options_8hpp" name="init_options.hpp" local="yes" imported="no">rclcpp/init_options.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::ContextAlreadyInitialized</class>
    <class kind="class">rclcpp::Context</class>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>std::vector&lt; Context::SharedPtr &gt;</type>
      <name>get_contexts</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>af73bf64bfc1b01f030012cd6d2e6e43c</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>default_context.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/contexts/</path>
    <filename>default__context_8hpp.html</filename>
    <includes id="context_8hpp" name="context.hpp" local="yes" imported="no">rclcpp/context.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::contexts::DefaultContext</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::contexts</namespace>
    <namespace>rclcpp::contexts::default_context</namespace>
    <member kind="typedef">
      <type>DefaultContext</type>
      <name>DefaultContext</name>
      <anchorfile>namespacerclcpp_1_1contexts_1_1default__context.html</anchorfile>
      <anchor>a4bb405cf2d9eac16028434a4e22dc3f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>DefaultContext::SharedPtr</type>
      <name>get_global_default_context</name>
      <anchorfile>namespacerclcpp_1_1contexts.html</anchorfile>
      <anchor>a18a2108b667f7e582db2c9ef216870a6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>DefaultContext::SharedPtr</type>
      <name>get_global_default_context</name>
      <anchorfile>namespacerclcpp_1_1contexts_1_1default__context.html</anchorfile>
      <anchor>af42b47432f5c680a63caab656e935fbb</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>create_client.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>create__client_8hpp.html</filename>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__services__interface_8hpp" name="node_services_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_services_interface.hpp</includes>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>rclcpp::Client&lt; ServiceT &gt;::SharedPtr</type>
      <name>create_client</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a24538aeccf07c4966c3192623d06440a</anchor>
      <arglist>(std::shared_ptr&lt; node_interfaces::NodeBaseInterface &gt; node_base, std::shared_ptr&lt; node_interfaces::NodeGraphInterface &gt; node_graph, std::shared_ptr&lt; node_interfaces::NodeServicesInterface &gt; node_services, const std::string &amp;service_name, const rmw_qos_profile_t &amp;qos_profile, rclcpp::CallbackGroup::SharedPtr group)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>create_publisher.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>create__publisher_8hpp.html</filename>
    <includes id="node__topics__interface_8hpp" name="node_topics_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_topics_interface.hpp</includes>
    <includes id="node__options_8hpp" name="node_options.hpp" local="yes" imported="no">rclcpp/node_options.hpp</includes>
    <includes id="publisher__factory_8hpp" name="publisher_factory.hpp" local="yes" imported="no">rclcpp/publisher_factory.hpp</includes>
    <includes id="publisher__options_8hpp" name="publisher_options.hpp" local="yes" imported="no">rclcpp/publisher_options.hpp</includes>
    <includes id="qos_8hpp" name="qos.hpp" local="yes" imported="no">rclcpp/qos.hpp</includes>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>std::shared_ptr&lt; PublisherT &gt;</type>
      <name>create_publisher</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a67010cd2562ccb9702e91904a7fb3e03</anchor>
      <arglist>(NodeT &amp;node, const std::string &amp;topic_name, const rclcpp::QoS &amp;qos, const rclcpp::PublisherOptionsWithAllocator&lt; AllocatorT &gt; &amp;options=(rclcpp::PublisherOptionsWithAllocator&lt; AllocatorT &gt;()))</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>create_service.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>create__service_8hpp.html</filename>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__services__interface_8hpp" name="node_services_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_services_interface.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>rclcpp::Service&lt; ServiceT &gt;::SharedPtr</type>
      <name>create_service</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a08f170aa649d228312e5eab2497042fb</anchor>
      <arglist>(std::shared_ptr&lt; node_interfaces::NodeBaseInterface &gt; node_base, std::shared_ptr&lt; node_interfaces::NodeServicesInterface &gt; node_services, const std::string &amp;service_name, CallbackT &amp;&amp;callback, const rmw_qos_profile_t &amp;qos_profile, rclcpp::CallbackGroup::SharedPtr group)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>create_subscription.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>create__subscription_8hpp.html</filename>
    <includes id="resolve__enable__topic__statistics_8hpp" name="resolve_enable_topic_statistics.hpp" local="yes" imported="no">rclcpp/detail/resolve_enable_topic_statistics.hpp</includes>
    <includes id="node__timers__interface_8hpp" name="node_timers_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_timers_interface.hpp</includes>
    <includes id="node__topics__interface_8hpp" name="node_topics_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_topics_interface.hpp</includes>
    <includes id="create__publisher_8hpp" name="create_publisher.hpp" local="yes" imported="no">rclcpp/create_publisher.hpp</includes>
    <includes id="create__timer_8hpp" name="create_timer.hpp" local="yes" imported="no">rclcpp/create_timer.hpp</includes>
    <includes id="qos_8hpp" name="qos.hpp" local="yes" imported="no">rclcpp/qos.hpp</includes>
    <includes id="subscription__factory_8hpp" name="subscription_factory.hpp" local="yes" imported="no">rclcpp/subscription_factory.hpp</includes>
    <includes id="subscription__options_8hpp" name="subscription_options.hpp" local="yes" imported="no">rclcpp/subscription_options.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="subscription__topic__statistics_8hpp" name="subscription_topic_statistics.hpp" local="yes" imported="no">rclcpp/topic_statistics/subscription_topic_statistics.hpp</includes>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>std::shared_ptr&lt; SubscriptionT &gt;</type>
      <name>create_subscription</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a90a603fbd9fd6cc6ec45e72e575fb182</anchor>
      <arglist>(NodeT &amp;&amp;node, const std::string &amp;topic_name, const rclcpp::QoS &amp;qos, CallbackT &amp;&amp;callback, const rclcpp::SubscriptionOptionsWithAllocator&lt; AllocatorT &gt; &amp;options=(rclcpp::SubscriptionOptionsWithAllocator&lt; AllocatorT &gt;()), typename MessageMemoryStrategyT::SharedPtr msg_mem_strat=(MessageMemoryStrategyT::create_default()))</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>create_timer.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>create__timer_8hpp.html</filename>
    <includes id="duration_8hpp" name="duration.hpp" local="yes" imported="no">rclcpp/duration.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__timers__interface_8hpp" name="node_timers_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_timers_interface.hpp</includes>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>rclcpp::TimerBase::SharedPtr</type>
      <name>create_timer</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a0c53f2add6632d67becd1885d26362c5</anchor>
      <arglist>(std::shared_ptr&lt; node_interfaces::NodeBaseInterface &gt; node_base, std::shared_ptr&lt; node_interfaces::NodeTimersInterface &gt; node_timers, rclcpp::Clock::SharedPtr clock, rclcpp::Duration period, CallbackT &amp;&amp;callback, rclcpp::CallbackGroup::SharedPtr group=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::TimerBase::SharedPtr</type>
      <name>create_timer</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>affb29d356bf00583b0d705b264b3b0ae</anchor>
      <arglist>(NodeT node, rclcpp::Clock::SharedPtr clock, rclcpp::Duration period, CallbackT &amp;&amp;callback, rclcpp::CallbackGroup::SharedPtr group=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::WallTimer&lt; CallbackT &gt;::SharedPtr</type>
      <name>create_wall_timer</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ad987fd245e31c6fd16d70d6ed1186189</anchor>
      <arglist>(std::chrono::duration&lt; DurationRepT, DurationT &gt; period, CallbackT callback, rclcpp::CallbackGroup::SharedPtr group, node_interfaces::NodeBaseInterface *node_base, node_interfaces::NodeTimersInterface *node_timers)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>mutex_two_priorities.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/detail/</path>
    <filename>mutex__two__priorities_8hpp.html</filename>
    <class kind="class">rclcpp::detail::MutexTwoPriorities</class>
    <class kind="class">rclcpp::detail::MutexTwoPriorities::HighPriorityLockable</class>
    <class kind="class">rclcpp::detail::MutexTwoPriorities::LowPriorityLockable</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::detail</namespace>
  </compound>
  <compound kind="file">
    <name>README.md</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/detail/</path>
    <filename>detail_2README_8md.html</filename>
  </compound>
  <compound kind="file">
    <name>README.md</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/experimental/</path>
    <filename>experimental_2README_8md.html</filename>
  </compound>
  <compound kind="file">
    <name>resolve_enable_topic_statistics.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/detail/</path>
    <filename>resolve__enable__topic__statistics_8hpp.html</filename>
    <includes id="topic__statistics__state_8hpp" name="topic_statistics_state.hpp" local="yes" imported="no">rclcpp/topic_statistics_state.hpp</includes>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::detail</namespace>
    <member kind="function">
      <type>bool</type>
      <name>resolve_enable_topic_statistics</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>ab4c905b03a1caa939ff486d085c199e1</anchor>
      <arglist>(const OptionsT &amp;options, const NodeBaseT &amp;node_base)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>resolve_intra_process_buffer_type.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/detail/</path>
    <filename>resolve__intra__process__buffer__type_8hpp.html</filename>
    <includes id="any__subscription__callback_8hpp" name="any_subscription_callback.hpp" local="yes" imported="no">rclcpp/any_subscription_callback.hpp</includes>
    <includes id="intra__process__buffer__type_8hpp" name="intra_process_buffer_type.hpp" local="yes" imported="no">rclcpp/intra_process_buffer_type.hpp</includes>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::detail</namespace>
    <member kind="function">
      <type>rclcpp::IntraProcessBufferType</type>
      <name>resolve_intra_process_buffer_type</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>a6301cf54892f0767beb71bab3ad190d3</anchor>
      <arglist>(const rclcpp::IntraProcessBufferType buffer_type, const rclcpp::AnySubscriptionCallback&lt; CallbackMessageT, AllocatorT &gt; &amp;any_subscription_callback)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>resolve_use_intra_process.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/detail/</path>
    <filename>resolve__use__intra__process_8hpp.html</filename>
    <includes id="intra__process__setting_8hpp" name="intra_process_setting.hpp" local="yes" imported="no">rclcpp/intra_process_setting.hpp</includes>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::detail</namespace>
    <member kind="function">
      <type>bool</type>
      <name>resolve_use_intra_process</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>ab4e9664da042c42723eba5fbf7c3f998</anchor>
      <arglist>(const OptionsT &amp;options, const NodeBaseT &amp;node_base)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>rmw_implementation_specific_payload.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/detail/</path>
    <filename>rmw__implementation__specific__payload_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::detail::RMWImplementationSpecificPayload</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::detail</namespace>
  </compound>
  <compound kind="file">
    <name>rmw_implementation_specific_publisher_payload.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/detail/</path>
    <filename>rmw__implementation__specific__publisher__payload_8hpp.html</filename>
    <includes id="rmw__implementation__specific__payload_8hpp" name="rmw_implementation_specific_payload.hpp" local="yes" imported="no">rclcpp/detail/rmw_implementation_specific_payload.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::detail::RMWImplementationSpecificPublisherPayload</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::detail</namespace>
  </compound>
  <compound kind="file">
    <name>rmw_implementation_specific_subscription_payload.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/detail/</path>
    <filename>rmw__implementation__specific__subscription__payload_8hpp.html</filename>
    <includes id="rmw__implementation__specific__payload_8hpp" name="rmw_implementation_specific_payload.hpp" local="yes" imported="no">rclcpp/detail/rmw_implementation_specific_payload.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::detail::RMWImplementationSpecificSubscriptionPayload</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::detail</namespace>
  </compound>
  <compound kind="file">
    <name>utilities.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/detail/</path>
    <filename>detail_2utilities_8hpp.html</filename>
    <includes id="detail_2utilities_8hpp" name="utilities.hpp" local="yes" imported="no">rclcpp/detail/utilities.hpp</includes>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::detail</namespace>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>get_unparsed_ros_arguments</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>a7cff1393aeeab323717e22fa15236da2</anchor>
      <arglist>(int argc, char const *const argv[], rcl_arguments_t *arguments, rcl_allocator_t allocator)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>utilities.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>utilities_8hpp.html</filename>
    <includes id="context_8hpp" name="context.hpp" local="yes" imported="no">rclcpp/context.hpp</includes>
    <includes id="init__options_8hpp" name="init_options.hpp" local="yes" imported="no">rclcpp/init_options.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>void</type>
      <name>init</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a2db29afebba8f677bc8660a45bb910bb</anchor>
      <arglist>(int argc, char const *const argv[], const InitOptions &amp;init_options=InitOptions())</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>install_signal_handlers</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a98aab08c64d725e46dee33ee5d705277</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>signal_handlers_installed</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a3944023ee7719c1b3eab2a1cd4e0f3f3</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>uninstall_signal_handlers</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ad74690adaed20915f98b36c0e93d3231</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>init_and_remove_ros_arguments</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ae27196203387b54cc4345b8d1303e45a</anchor>
      <arglist>(int argc, char const *const argv[], const InitOptions &amp;init_options=InitOptions())</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>remove_ros_arguments</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ad4968a767791995c011c57be22cd40c8</anchor>
      <arglist>(int argc, char const *const argv[])</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>ok</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>adbe8ffd2b1769e897f2c50d560812b43</anchor>
      <arglist>(rclcpp::Context::SharedPtr context=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_initialized</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ac9460447dae147a331cab70668a7ddd2</anchor>
      <arglist>(rclcpp::Context::SharedPtr context=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>shutdown</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a493714a679d1591142800416a286689f</anchor>
      <arglist>(rclcpp::Context::SharedPtr context=nullptr, const std::string &amp;reason=&quot;user called rclcpp::shutdown()&quot;)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>on_shutdown</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a01e2c223964ccca7ede393af47fac025</anchor>
      <arglist>(std::function&lt; void()&gt; callback, rclcpp::Context::SharedPtr context=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>sleep_for</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ac305329e4e97948d4bb216e894caa4ae</anchor>
      <arglist>(const std::chrono::nanoseconds &amp;nanoseconds, rclcpp::Context::SharedPtr context=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>add_will_overflow</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>af238f376176cf3da48adc46b94d29a6a</anchor>
      <arglist>(const T x, const T y)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>add_will_underflow</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a26f62ce86fabd324005231d8d89a8294</anchor>
      <arglist>(const T x, const T y)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>sub_will_overflow</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa7c63f4d5146a9054f3b2d8b9ac2070f</anchor>
      <arglist>(const T x, const T y)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>sub_will_underflow</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>af340ddf3a7b82a7a5a4808740a039e69</anchor>
      <arglist>(const T x, const T y)</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_c_string</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1fdf023cdc167cb3d4d353353cd4cced</anchor>
      <arglist>(const char *string_in)</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_c_string</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a7743e6192ef035115abdeeb90227c45c</anchor>
      <arglist>(const std::string &amp;string_in)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>duration.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>duration_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::Duration</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>event.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>event_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::Event</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>exceptions.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/exceptions/</path>
    <filename>exceptions_2exceptions_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::exceptions::InvalidNodeError</class>
    <class kind="class">rclcpp::exceptions::NameValidationError</class>
    <class kind="class">rclcpp::exceptions::InvalidNodeNameError</class>
    <class kind="class">rclcpp::exceptions::InvalidNamespaceError</class>
    <class kind="class">rclcpp::exceptions::InvalidTopicNameError</class>
    <class kind="class">rclcpp::exceptions::InvalidServiceNameError</class>
    <class kind="class">rclcpp::exceptions::UnimplementedError</class>
    <class kind="class">rclcpp::exceptions::RCLErrorBase</class>
    <class kind="class">rclcpp::exceptions::RCLError</class>
    <class kind="class">rclcpp::exceptions::RCLBadAlloc</class>
    <class kind="class">rclcpp::exceptions::RCLInvalidArgument</class>
    <class kind="class">rclcpp::exceptions::RCLInvalidROSArgsError</class>
    <class kind="class">rclcpp::exceptions::UnknownROSArgsError</class>
    <class kind="class">rclcpp::exceptions::InvalidEventError</class>
    <class kind="class">rclcpp::exceptions::EventNotRegisteredError</class>
    <class kind="class">rclcpp::exceptions::InvalidParametersException</class>
    <class kind="class">rclcpp::exceptions::InvalidParameterValueException</class>
    <class kind="class">rclcpp::exceptions::InvalidParameterTypeException</class>
    <class kind="class">rclcpp::exceptions::ParameterAlreadyDeclaredException</class>
    <class kind="class">rclcpp::exceptions::ParameterNotDeclaredException</class>
    <class kind="class">rclcpp::exceptions::ParameterImmutableException</class>
    <class kind="class">rclcpp::exceptions::ParameterModifiedInCallbackException</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::exceptions</namespace>
    <member kind="function">
      <type>void</type>
      <name>throw_from_rcl_error</name>
      <anchorfile>namespacerclcpp_1_1exceptions.html</anchorfile>
      <anchor>ad1d5563cfff9273f531a82c1dde13516</anchor>
      <arglist>(rcl_ret_t ret, const std::string &amp;prefix=&quot;&quot;, const rcl_error_state_t *error_state=nullptr, void(*reset_error)()=rcl_reset_error)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>exceptions.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>exceptions_8hpp.html</filename>
    <includes id="exceptions_2exceptions_8hpp" name="exceptions.hpp" local="yes" imported="no">rclcpp/exceptions/exceptions.hpp</includes>
  </compound>
  <compound kind="file">
    <name>executor.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>executor_8hpp.html</filename>
    <includes id="default__context_8hpp" name="default_context.hpp" local="yes" imported="no">rclcpp/contexts/default_context.hpp</includes>
    <includes id="executor__options_8hpp" name="executor_options.hpp" local="yes" imported="no">rclcpp/executor_options.hpp</includes>
    <includes id="future__return__code_8hpp" name="future_return_code.hpp" local="yes" imported="no">rclcpp/future_return_code.hpp</includes>
    <includes id="memory__strategies_8hpp" name="memory_strategies.hpp" local="yes" imported="no">rclcpp/memory_strategies.hpp</includes>
    <includes id="memory__strategy_8hpp" name="memory_strategy.hpp" local="yes" imported="no">rclcpp/memory_strategy.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="utilities_8hpp" name="utilities.hpp" local="yes" imported="no">rclcpp/utilities.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="scope__exit_8hpp" name="scope_exit.hpp" local="yes" imported="no">rclcpp/scope_exit.hpp</includes>
    <class kind="class">rclcpp::Executor</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::executor</namespace>
    <member kind="typedef">
      <type>rclcpp::Executor</type>
      <name>Executor</name>
      <anchorfile>namespacerclcpp_1_1executor.html</anchorfile>
      <anchor>aa7ab7535de7e4a9f4c9dc9ca3333ede3</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>executor_options.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>executor__options_8hpp.html</filename>
    <includes id="context_8hpp" name="context.hpp" local="yes" imported="no">rclcpp/context.hpp</includes>
    <includes id="default__context_8hpp" name="default_context.hpp" local="yes" imported="no">rclcpp/contexts/default_context.hpp</includes>
    <includes id="memory__strategies_8hpp" name="memory_strategies.hpp" local="yes" imported="no">rclcpp/memory_strategies.hpp</includes>
    <includes id="memory__strategy_8hpp" name="memory_strategy.hpp" local="yes" imported="no">rclcpp/memory_strategy.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="struct">rclcpp::ExecutorOptions</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::executor</namespace>
    <member kind="typedef">
      <type>ExecutorOptions</type>
      <name>ExecutorArgs</name>
      <anchorfile>namespacerclcpp_1_1executor.html</anchorfile>
      <anchor>a35debac90796a7551a850e56ad96206e</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>rclcpp::ExecutorOptions</type>
      <name>create_default_executor_arguments</name>
      <anchorfile>namespacerclcpp_1_1executor.html</anchorfile>
      <anchor>adfc3aa74cdc0b03340727f779cb58eb9</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>executors.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>executors_8hpp.html</filename>
    <includes id="multi__threaded__executor_8hpp" name="multi_threaded_executor.hpp" local="yes" imported="no">rclcpp/executors/multi_threaded_executor.hpp</includes>
    <includes id="single__threaded__executor_8hpp" name="single_threaded_executor.hpp" local="yes" imported="no">rclcpp/executors/single_threaded_executor.hpp</includes>
    <includes id="static__single__threaded__executor_8hpp" name="static_single_threaded_executor.hpp" local="yes" imported="no">rclcpp/executors/static_single_threaded_executor.hpp</includes>
    <includes id="node_8hpp" name="node.hpp" local="yes" imported="no">rclcpp/node.hpp</includes>
    <includes id="utilities_8hpp" name="utilities.hpp" local="yes" imported="no">rclcpp/utilities.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::executors</namespace>
    <member kind="function">
      <type>void</type>
      <name>spin_some</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ad48c7a9cc4fa34989a0849d708d8f7de</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin_some</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a5e16488d62cc48e5520101f9f4f4102a</anchor>
      <arglist>(rclcpp::Node::SharedPtr node_ptr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a21e13577f5bcc5992de1d7dd08d8652b</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a4bb335bc95a6fa8546a6bfcfd087eb57</anchor>
      <arglist>(rclcpp::Node::SharedPtr node_ptr)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::FutureReturnCode</type>
      <name>spin_node_until_future_complete</name>
      <anchorfile>namespacerclcpp_1_1executors.html</anchorfile>
      <anchor>a1c80eee179a16b9d564ddb5dba55c625</anchor>
      <arglist>(rclcpp::Executor &amp;executor, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, const std::shared_future&lt; ResponseT &gt; &amp;future, std::chrono::duration&lt; TimeRepT, TimeT &gt; timeout=std::chrono::duration&lt; TimeRepT, TimeT &gt;(-1))</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::FutureReturnCode</type>
      <name>spin_node_until_future_complete</name>
      <anchorfile>namespacerclcpp_1_1executors.html</anchorfile>
      <anchor>a945cba4bc9c68284248b2bd91d168c42</anchor>
      <arglist>(rclcpp::Executor &amp;executor, std::shared_ptr&lt; NodeT &gt; node_ptr, const std::shared_future&lt; ResponseT &gt; &amp;future, std::chrono::duration&lt; TimeRepT, TimeT &gt; timeout=std::chrono::duration&lt; TimeRepT, TimeT &gt;(-1))</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::FutureReturnCode</type>
      <name>spin_until_future_complete</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a702eef83e9bdbaabbc89e426668d808c</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, const std::shared_future&lt; FutureT &gt; &amp;future, std::chrono::duration&lt; TimeRepT, TimeT &gt; timeout=std::chrono::duration&lt; TimeRepT, TimeT &gt;(-1))</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::FutureReturnCode</type>
      <name>spin_until_future_complete</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a23b2eb933df4c6b35dc5ffa01b78bcfc</anchor>
      <arglist>(std::shared_ptr&lt; NodeT &gt; node_ptr, const std::shared_future&lt; FutureT &gt; &amp;future, std::chrono::duration&lt; TimeRepT, TimeT &gt; timeout=std::chrono::duration&lt; TimeRepT, TimeT &gt;(-1))</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>multi_threaded_executor.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/executors/</path>
    <filename>multi__threaded__executor_8hpp.html</filename>
    <includes id="mutex__two__priorities_8hpp" name="mutex_two_priorities.hpp" local="yes" imported="no">rclcpp/detail/mutex_two_priorities.hpp</includes>
    <includes id="executor_8hpp" name="executor.hpp" local="yes" imported="no">rclcpp/executor.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="memory__strategies_8hpp" name="memory_strategies.hpp" local="yes" imported="no">rclcpp/memory_strategies.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::executors::MultiThreadedExecutor</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::executors</namespace>
  </compound>
  <compound kind="file">
    <name>single_threaded_executor.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/executors/</path>
    <filename>single__threaded__executor_8hpp.html</filename>
    <includes id="executor_8hpp" name="executor.hpp" local="yes" imported="no">rclcpp/executor.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="memory__strategies_8hpp" name="memory_strategies.hpp" local="yes" imported="no">rclcpp/memory_strategies.hpp</includes>
    <includes id="node_8hpp" name="node.hpp" local="yes" imported="no">rclcpp/node.hpp</includes>
    <includes id="utilities_8hpp" name="utilities.hpp" local="yes" imported="no">rclcpp/utilities.hpp</includes>
    <includes id="rate_8hpp" name="rate.hpp" local="yes" imported="no">rclcpp/rate.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::executors::SingleThreadedExecutor</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::executors</namespace>
  </compound>
  <compound kind="file">
    <name>static_executor_entities_collector.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/executors/</path>
    <filename>static__executor__entities__collector_8hpp.html</filename>
    <includes id="executable__list_8hpp" name="executable_list.hpp" local="yes" imported="no">rclcpp/experimental/executable_list.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="memory__strategy_8hpp" name="memory_strategy.hpp" local="yes" imported="no">rclcpp/memory_strategy.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::executors::StaticExecutorEntitiesCollector</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::executors</namespace>
  </compound>
  <compound kind="file">
    <name>static_single_threaded_executor.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/executors/</path>
    <filename>static__single__threaded__executor_8hpp.html</filename>
    <includes id="executor_8hpp" name="executor.hpp" local="yes" imported="no">rclcpp/executor.hpp</includes>
    <includes id="static__executor__entities__collector_8hpp" name="static_executor_entities_collector.hpp" local="yes" imported="no">rclcpp/executors/static_executor_entities_collector.hpp</includes>
    <includes id="executable__list_8hpp" name="executable_list.hpp" local="yes" imported="no">rclcpp/experimental/executable_list.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="memory__strategies_8hpp" name="memory_strategies.hpp" local="yes" imported="no">rclcpp/memory_strategies.hpp</includes>
    <includes id="node_8hpp" name="node.hpp" local="yes" imported="no">rclcpp/node.hpp</includes>
    <includes id="rate_8hpp" name="rate.hpp" local="yes" imported="no">rclcpp/rate.hpp</includes>
    <includes id="utilities_8hpp" name="utilities.hpp" local="yes" imported="no">rclcpp/utilities.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::executors::StaticSingleThreadedExecutor</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::executors</namespace>
  </compound>
  <compound kind="file">
    <name>expand_topic_or_service_name.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>expand__topic__or__service__name_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>std::string</type>
      <name>expand_topic_or_service_name</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1855b992b1b77f1ed75bab5192aaf2bd</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;node_name, const std::string &amp;namespace_, bool is_service=false)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>buffer_implementation_base.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/experimental/buffers/</path>
    <filename>buffer__implementation__base_8hpp.html</filename>
    <class kind="class">rclcpp::experimental::buffers::BufferImplementationBase</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::experimental</namespace>
    <namespace>rclcpp::experimental::buffers</namespace>
  </compound>
  <compound kind="file">
    <name>intra_process_buffer.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/experimental/buffers/</path>
    <filename>intra__process__buffer_8hpp.html</filename>
    <includes id="allocator__common_8hpp" name="allocator_common.hpp" local="yes" imported="no">rclcpp/allocator/allocator_common.hpp</includes>
    <includes id="allocator__deleter_8hpp" name="allocator_deleter.hpp" local="yes" imported="no">rclcpp/allocator/allocator_deleter.hpp</includes>
    <includes id="buffer__implementation__base_8hpp" name="buffer_implementation_base.hpp" local="yes" imported="no">rclcpp/experimental/buffers/buffer_implementation_base.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <class kind="class">rclcpp::experimental::buffers::IntraProcessBufferBase</class>
    <class kind="class">rclcpp::experimental::buffers::IntraProcessBuffer</class>
    <class kind="class">rclcpp::experimental::buffers::TypedIntraProcessBuffer</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::experimental</namespace>
    <namespace>rclcpp::experimental::buffers</namespace>
  </compound>
  <compound kind="file">
    <name>ring_buffer_implementation.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/experimental/buffers/</path>
    <filename>ring__buffer__implementation_8hpp.html</filename>
    <includes id="buffer__implementation__base_8hpp" name="buffer_implementation_base.hpp" local="yes" imported="no">rclcpp/experimental/buffers/buffer_implementation_base.hpp</includes>
    <includes id="logger_8hpp" name="logger.hpp" local="yes" imported="no">rclcpp/logger.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::experimental::buffers::RingBufferImplementation</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::experimental</namespace>
    <namespace>rclcpp::experimental::buffers</namespace>
  </compound>
  <compound kind="file">
    <name>create_intra_process_buffer.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/experimental/</path>
    <filename>create__intra__process__buffer_8hpp.html</filename>
    <includes id="intra__process__buffer_8hpp" name="intra_process_buffer.hpp" local="yes" imported="no">rclcpp/experimental/buffers/intra_process_buffer.hpp</includes>
    <includes id="ring__buffer__implementation_8hpp" name="ring_buffer_implementation.hpp" local="yes" imported="no">rclcpp/experimental/buffers/ring_buffer_implementation.hpp</includes>
    <includes id="intra__process__buffer__type_8hpp" name="intra_process_buffer_type.hpp" local="yes" imported="no">rclcpp/intra_process_buffer_type.hpp</includes>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::experimental</namespace>
    <member kind="function">
      <type>rclcpp::experimental::buffers::IntraProcessBuffer&lt; MessageT, Alloc, Deleter &gt;::UniquePtr</type>
      <name>create_intra_process_buffer</name>
      <anchorfile>namespacerclcpp_1_1experimental.html</anchorfile>
      <anchor>af620d29730460d7f82e51157e1dff688</anchor>
      <arglist>(IntraProcessBufferType buffer_type, rmw_qos_profile_t qos, std::shared_ptr&lt; Alloc &gt; allocator)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>executable_list.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/experimental/</path>
    <filename>executable__list_8hpp.html</filename>
    <includes id="client_8hpp" name="client.hpp" local="yes" imported="no">rclcpp/client.hpp</includes>
    <includes id="service_8hpp" name="service.hpp" local="yes" imported="no">rclcpp/service.hpp</includes>
    <includes id="subscription__base_8hpp" name="subscription_base.hpp" local="yes" imported="no">rclcpp/subscription_base.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::experimental::ExecutableList</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::experimental</namespace>
  </compound>
  <compound kind="file">
    <name>intra_process_manager.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/experimental/</path>
    <filename>intra__process__manager_8hpp.html</filename>
    <includes id="allocator__deleter_8hpp" name="allocator_deleter.hpp" local="yes" imported="no">rclcpp/allocator/allocator_deleter.hpp</includes>
    <includes id="subscription__intra__process_8hpp" name="subscription_intra_process.hpp" local="yes" imported="no">rclcpp/experimental/subscription_intra_process.hpp</includes>
    <includes id="subscription__intra__process__base_8hpp" name="subscription_intra_process_base.hpp" local="yes" imported="no">rclcpp/experimental/subscription_intra_process_base.hpp</includes>
    <includes id="logger_8hpp" name="logger.hpp" local="yes" imported="no">rclcpp/logger.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="publisher__base_8hpp" name="publisher_base.hpp" local="yes" imported="no">rclcpp/publisher_base.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::experimental::IntraProcessManager</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::experimental</namespace>
  </compound>
  <compound kind="file">
    <name>subscription_intra_process.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/experimental/</path>
    <filename>subscription__intra__process_8hpp.html</filename>
    <includes id="any__subscription__callback_8hpp" name="any_subscription_callback.hpp" local="yes" imported="no">rclcpp/any_subscription_callback.hpp</includes>
    <includes id="intra__process__buffer_8hpp" name="intra_process_buffer.hpp" local="yes" imported="no">rclcpp/experimental/buffers/intra_process_buffer.hpp</includes>
    <includes id="create__intra__process__buffer_8hpp" name="create_intra_process_buffer.hpp" local="yes" imported="no">rclcpp/experimental/create_intra_process_buffer.hpp</includes>
    <includes id="subscription__intra__process__base_8hpp" name="subscription_intra_process_base.hpp" local="yes" imported="no">rclcpp/experimental/subscription_intra_process_base.hpp</includes>
    <includes id="type__support__decl_8hpp" name="type_support_decl.hpp" local="yes" imported="no">rclcpp/type_support_decl.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::experimental::SubscriptionIntraProcess</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::experimental</namespace>
  </compound>
  <compound kind="file">
    <name>subscription_intra_process_base.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/experimental/</path>
    <filename>subscription__intra__process__base_8hpp.html</filename>
    <includes id="type__support__decl_8hpp" name="type_support_decl.hpp" local="yes" imported="no">rclcpp/type_support_decl.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::experimental::SubscriptionIntraProcessBase</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::experimental</namespace>
  </compound>
  <compound kind="file">
    <name>function_traits.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>function__traits_8hpp.html</filename>
    <class kind="struct">rclcpp::function_traits::tuple_tail</class>
    <class kind="struct">rclcpp::function_traits::tuple_tail&lt; std::tuple&lt; Head, Tail ... &gt; &gt;</class>
    <class kind="struct">rclcpp::function_traits::function_traits</class>
    <class kind="struct">rclcpp::function_traits::function_traits&lt; ReturnTypeT(Args ...)&gt;</class>
    <class kind="struct">rclcpp::function_traits::function_traits&lt; ReturnTypeT(*)(Args ...)&gt;</class>
    <class kind="struct">rclcpp::function_traits::function_traits&lt; ReturnTypeT(ClassT::*)(Args ...) const &gt;</class>
    <class kind="struct">rclcpp::function_traits::function_traits&lt; FunctionT &amp; &gt;</class>
    <class kind="struct">rclcpp::function_traits::function_traits&lt; FunctionT &amp;&amp; &gt;</class>
    <class kind="struct">rclcpp::function_traits::arity_comparator</class>
    <class kind="struct">rclcpp::function_traits::check_arguments</class>
    <class kind="struct">rclcpp::function_traits::same_arguments</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::function_traits</namespace>
    <member kind="variable">
      <type></type>
      <name>__pad0__</name>
      <anchorfile>namespacerclcpp_1_1function__traits.html</anchorfile>
      <anchor>ae3374ee3b21cea1d76f0f68d52a43521</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type></type>
      <name>__pad1__</name>
      <anchorfile>namespacerclcpp_1_1function__traits.html</anchorfile>
      <anchor>a40d2b02c8eca43d060b10b461fe58a42</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type></type>
      <name>__pad2__</name>
      <anchorfile>namespacerclcpp_1_1function__traits.html</anchorfile>
      <anchor>a9b406c4834621946f1fb16ad4400eb4e</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>future_return_code.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>future__return__code_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::executor</namespace>
    <member kind="typedef">
      <type>FutureReturnCode</type>
      <name>FutureReturnCode</name>
      <anchorfile>namespacerclcpp_1_1executor.html</anchorfile>
      <anchor>a3b8f8bd56c33a1c2f0ab3d6fe5d4e089</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>FutureReturnCode</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a7b4ff5f1e516740d7e11ea97fe6f5532</anchor>
      <arglist></arglist>
      <enumvalue file="namespacerclcpp.html" anchor="a7b4ff5f1e516740d7e11ea97fe6f5532ad0749aaba8b833466dfcbb0428e4f89c">SUCCESS</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a7b4ff5f1e516740d7e11ea97fe6f5532a658f2cadfdf09b6046246e9314f7cd43">INTERRUPTED</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a7b4ff5f1e516740d7e11ea97fe6f5532a070a0fb40f6c308ab544b227660aadff">TIMEOUT</enumvalue>
    </member>
    <member kind="function">
      <type>std::ostream &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a9a474b200d67e8a5988daaeb8980fb8b</anchor>
      <arglist>(std::ostream &amp;os, const FutureReturnCode &amp;future_return_code)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a0a11b0ee43bb55e4eca3a79866406e5e</anchor>
      <arglist>(const FutureReturnCode &amp;future_return_code)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>namespacerclcpp_1_1executor.html</anchorfile>
      <anchor>a402969c33929fa7d75f95f89514a4df9</anchor>
      <arglist>(const rclcpp::FutureReturnCode &amp;future_return_code)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>graph_listener.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>graph__listener_8hpp.html</filename>
    <includes id="context_8hpp" name="context.hpp" local="yes" imported="no">rclcpp/context.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__graph__interface_8hpp" name="node_graph_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_graph_interface.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::graph_listener::GraphListenerShutdownError</class>
    <class kind="class">rclcpp::graph_listener::NodeAlreadyAddedError</class>
    <class kind="class">rclcpp::graph_listener::NodeNotFoundError</class>
    <class kind="class">rclcpp::graph_listener::GraphListener</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::graph_listener</namespace>
  </compound>
  <compound kind="file">
    <name>guard_condition.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>guard__condition_8hpp.html</filename>
    <includes id="context_8hpp" name="context.hpp" local="yes" imported="no">rclcpp/context.hpp</includes>
    <includes id="default__context_8hpp" name="default_context.hpp" local="yes" imported="no">rclcpp/contexts/default_context.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::GuardCondition</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>init_options.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>init__options_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::InitOptions</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>intra_process_buffer_type.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>intra__process__buffer__type_8hpp.html</filename>
    <namespace>rclcpp</namespace>
    <member kind="enumeration">
      <type></type>
      <name>IntraProcessBufferType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a53c5da68d5964c6bd7894afe4a76a92b</anchor>
      <arglist></arglist>
      <enumvalue file="namespacerclcpp.html" anchor="a53c5da68d5964c6bd7894afe4a76a92ba309589a0fa2f1ec5e6b286ac5e8b6ac8">SharedPtr</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a53c5da68d5964c6bd7894afe4a76a92ba8c4322b401772928915c5a3ada1304d5">UniquePtr</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a53c5da68d5964c6bd7894afe4a76a92ba50c8a640832cc7a38533a3d5d3da60df">CallbackDefault</enumvalue>
    </member>
  </compound>
  <compound kind="file">
    <name>intra_process_setting.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>intra__process__setting_8hpp.html</filename>
    <namespace>rclcpp</namespace>
    <member kind="enumeration">
      <type></type>
      <name>IntraProcessSetting</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a2ebc5fdfcb7e3f025f8ebc2fe1d10fe5</anchor>
      <arglist></arglist>
      <enumvalue file="namespacerclcpp.html" anchor="a2ebc5fdfcb7e3f025f8ebc2fe1d10fe5a2faec1f9f8cc7f8f40d521c4dd574f49">Enable</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a2ebc5fdfcb7e3f025f8ebc2fe1d10fe5abcfaccebf745acfd5e75351095a5394a">Disable</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a2ebc5fdfcb7e3f025f8ebc2fe1d10fe5a5e7bd84eadd196f52e8320680fa1c7cf">NodeDefault</enumvalue>
    </member>
  </compound>
  <compound kind="file">
    <name>loaned_message.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>loaned__message_8hpp.html</filename>
    <includes id="allocator__common_8hpp" name="allocator_common.hpp" local="yes" imported="no">rclcpp/allocator/allocator_common.hpp</includes>
    <includes id="publisher__base_8hpp" name="publisher_base.hpp" local="yes" imported="no">rclcpp/publisher_base.hpp</includes>
    <class kind="class">rclcpp::LoanedMessage</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>logger.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>logger_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::Logger</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_LOGGING_ENABLED</name>
      <anchorfile>logger_8hpp.html</anchorfile>
      <anchor>a67cbbd075473dfa12cd3f434fe9320ad</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>Logger</type>
      <name>get_logger</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ae7295751947c08312aa69f45fd673171</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>Logger</type>
      <name>get_node_logger</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a32cf150e9157d8ae52206bbb7f1a9310</anchor>
      <arglist>(const rcl_node_t *node)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>macros.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>macros_8hpp.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_DISABLE_COPY</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>a0a7b455740daed4f09fe358316b3f582</anchor>
      <arglist>(...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_SMART_PTR_DEFINITIONS</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>acf7fe89712fbf4410ed29e86b081deaf</anchor>
      <arglist>(...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>ad8b6db2e8a167744be0edad2e07b19f3</anchor>
      <arglist>(...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_SMART_PTR_ALIASES_ONLY</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>acdbfb9a16d2543219fa74417762eb9ee</anchor>
      <arglist>(...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>__RCLCPP_SHARED_PTR_ALIAS</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>aa28e069950bf0d5d5376437476308211</anchor>
      <arglist>(...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>__RCLCPP_MAKE_SHARED_DEFINITION</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>ac6100ecf30618979271e924ae00faafe</anchor>
      <arglist>(...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_SHARED_PTR_DEFINITIONS</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>a81544d33068b1423ffd418254614cc1b</anchor>
      <arglist>(...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>__RCLCPP_WEAK_PTR_ALIAS</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>ab4088e27ee18722b5a3b056bfc4fcce3</anchor>
      <arglist>(...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_WEAK_PTR_DEFINITIONS</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>a150f4aee662ba3cf3257f2b9310dc301</anchor>
      <arglist>(...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>__RCLCPP_UNIQUE_PTR_ALIAS</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>ad3104ba8b9f139139f855bb001091a8c</anchor>
      <arglist>(...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>__RCLCPP_MAKE_UNIQUE_DEFINITION</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>a72e5f97814faddba1a3919eaeab40718</anchor>
      <arglist>(...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_UNIQUE_PTR_DEFINITIONS</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>a74b2138ad12edbfb75edd98b73c37895</anchor>
      <arglist>(...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_STRING_JOIN</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>abaf6b89d68d61ccc8295f30c6e82bfaf</anchor>
      <arglist>(arg1, arg2)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_DO_STRING_JOIN</name>
      <anchorfile>macros_8hpp.html</anchorfile>
      <anchor>a8e5661370a885f08c53c44d78bf2da02</anchor>
      <arglist>(arg1, arg2)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>memory_strategies.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>memory__strategies_8hpp.html</filename>
    <includes id="memory__strategy_8hpp" name="memory_strategy.hpp" local="yes" imported="no">rclcpp/memory_strategy.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::memory_strategies</namespace>
    <member kind="function">
      <type>memory_strategy::MemoryStrategy::SharedPtr</type>
      <name>create_default_strategy</name>
      <anchorfile>namespacerclcpp_1_1memory__strategies.html</anchorfile>
      <anchor>a4d6a626858dbf7a3cc0936897764bfe9</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>memory_strategy.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>memory__strategy_8hpp.html</filename>
    <includes id="any__executable_8hpp" name="any_executable.hpp" local="yes" imported="no">rclcpp/any_executable.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::memory_strategy::MemoryStrategy</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::memory_strategy</namespace>
  </compound>
  <compound kind="file">
    <name>message_info.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>message__info_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::MessageInfo</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>message_memory_strategy.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>message__memory__strategy_8hpp.html</filename>
    <includes id="allocator__common_8hpp" name="allocator_common.hpp" local="yes" imported="no">rclcpp/allocator/allocator_common.hpp</includes>
    <includes id="exceptions_8hpp" name="exceptions.hpp" local="yes" imported="no">rclcpp/exceptions.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="serialized__message_8hpp" name="serialized_message.hpp" local="yes" imported="no">rclcpp/serialized_message.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::message_memory_strategy::MessageMemoryStrategy</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::message_memory_strategy</namespace>
  </compound>
  <compound kind="file">
    <name>node.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>node_8hpp.html</filename>
    <includes id="callback__group_8hpp" name="callback_group.hpp" local="yes" imported="no">rclcpp/callback_group.hpp</includes>
    <includes id="client_8hpp" name="client.hpp" local="yes" imported="no">rclcpp/client.hpp</includes>
    <includes id="clock_8hpp" name="clock.hpp" local="yes" imported="no">rclcpp/clock.hpp</includes>
    <includes id="context_8hpp" name="context.hpp" local="yes" imported="no">rclcpp/context.hpp</includes>
    <includes id="event_8hpp" name="event.hpp" local="yes" imported="no">rclcpp/event.hpp</includes>
    <includes id="logger_8hpp" name="logger.hpp" local="yes" imported="no">rclcpp/logger.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="message__memory__strategy_8hpp" name="message_memory_strategy.hpp" local="yes" imported="no">rclcpp/message_memory_strategy.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__clock__interface_8hpp" name="node_clock_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_clock_interface.hpp</includes>
    <includes id="node__graph__interface_8hpp" name="node_graph_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_graph_interface.hpp</includes>
    <includes id="node__logging__interface_8hpp" name="node_logging_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_logging_interface.hpp</includes>
    <includes id="node__parameters__interface_8hpp" name="node_parameters_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_parameters_interface.hpp</includes>
    <includes id="node__services__interface_8hpp" name="node_services_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_services_interface.hpp</includes>
    <includes id="node__time__source__interface_8hpp" name="node_time_source_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_time_source_interface.hpp</includes>
    <includes id="node__timers__interface_8hpp" name="node_timers_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_timers_interface.hpp</includes>
    <includes id="node__topics__interface_8hpp" name="node_topics_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_topics_interface.hpp</includes>
    <includes id="node__waitables__interface_8hpp" name="node_waitables_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_waitables_interface.hpp</includes>
    <includes id="node__options_8hpp" name="node_options.hpp" local="yes" imported="no">rclcpp/node_options.hpp</includes>
    <includes id="parameter_8hpp" name="parameter.hpp" local="yes" imported="no">rclcpp/parameter.hpp</includes>
    <includes id="publisher_8hpp" name="publisher.hpp" local="yes" imported="no">rclcpp/publisher.hpp</includes>
    <includes id="publisher__options_8hpp" name="publisher_options.hpp" local="yes" imported="no">rclcpp/publisher_options.hpp</includes>
    <includes id="qos_8hpp" name="qos.hpp" local="yes" imported="no">rclcpp/qos.hpp</includes>
    <includes id="service_8hpp" name="service.hpp" local="yes" imported="no">rclcpp/service.hpp</includes>
    <includes id="subscription_8hpp" name="subscription.hpp" local="yes" imported="no">rclcpp/subscription.hpp</includes>
    <includes id="subscription__options_8hpp" name="subscription_options.hpp" local="yes" imported="no">rclcpp/subscription_options.hpp</includes>
    <includes id="subscription__traits_8hpp" name="subscription_traits.hpp" local="yes" imported="no">rclcpp/subscription_traits.hpp</includes>
    <includes id="time_8hpp" name="time.hpp" local="yes" imported="no">rclcpp/time.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="node__impl_8hpp" name="node_impl.hpp" local="yes" imported="no">node_impl.hpp</includes>
    <class kind="class">rclcpp::Node</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>node_impl.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>node__impl_8hpp.html</filename>
    <includes id="default__context_8hpp" name="default_context.hpp" local="yes" imported="no">rclcpp/contexts/default_context.hpp</includes>
    <includes id="create__client_8hpp" name="create_client.hpp" local="yes" imported="no">rclcpp/create_client.hpp</includes>
    <includes id="create__publisher_8hpp" name="create_publisher.hpp" local="yes" imported="no">rclcpp/create_publisher.hpp</includes>
    <includes id="create__service_8hpp" name="create_service.hpp" local="yes" imported="no">rclcpp/create_service.hpp</includes>
    <includes id="create__timer_8hpp" name="create_timer.hpp" local="yes" imported="no">rclcpp/create_timer.hpp</includes>
    <includes id="create__subscription_8hpp" name="create_subscription.hpp" local="yes" imported="no">rclcpp/create_subscription.hpp</includes>
    <includes id="resolve__enable__topic__statistics_8hpp" name="resolve_enable_topic_statistics.hpp" local="yes" imported="no">rclcpp/detail/resolve_enable_topic_statistics.hpp</includes>
    <includes id="parameter_8hpp" name="parameter.hpp" local="yes" imported="no">rclcpp/parameter.hpp</includes>
    <includes id="qos_8hpp" name="qos.hpp" local="yes" imported="no">rclcpp/qos.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="type__support__decl_8hpp" name="type_support_decl.hpp" local="yes" imported="no">rclcpp/type_support_decl.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="node_8hpp" name="node.hpp" local="yes" imported="no">node.hpp</includes>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>RCLCPP_LOCAL std::string</type>
      <name>extend_name_with_sub_namespace</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a575513c28b9d018ab59102e53f49f57d</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;sub_namespace)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>node_base.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__base_8hpp.html</filename>
    <includes id="context_8hpp" name="context.hpp" local="yes" imported="no">rclcpp/context.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeBase</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_base_interface.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__base__interface_8hpp.html</filename>
    <includes id="callback__group_8hpp" name="callback_group.hpp" local="yes" imported="no">rclcpp/callback_group.hpp</includes>
    <includes id="context_8hpp" name="context.hpp" local="yes" imported="no">rclcpp/context.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeBaseInterface</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_clock.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__clock_8hpp.html</filename>
    <includes id="clock_8hpp" name="clock.hpp" local="yes" imported="no">rclcpp/clock.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__clock__interface_8hpp" name="node_clock_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_clock_interface.hpp</includes>
    <includes id="node__graph__interface_8hpp" name="node_graph_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_graph_interface.hpp</includes>
    <includes id="node__logging__interface_8hpp" name="node_logging_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_logging_interface.hpp</includes>
    <includes id="node__services__interface_8hpp" name="node_services_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_services_interface.hpp</includes>
    <includes id="node__topics__interface_8hpp" name="node_topics_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_topics_interface.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeClock</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_clock_interface.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__clock__interface_8hpp.html</filename>
    <includes id="clock_8hpp" name="clock.hpp" local="yes" imported="no">rclcpp/clock.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeClockInterface</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_graph.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__graph_8hpp.html</filename>
    <includes id="event_8hpp" name="event.hpp" local="yes" imported="no">rclcpp/event.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__graph__interface_8hpp" name="node_graph_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_graph_interface.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeGraph</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::graph_listener</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_graph_interface.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__graph__interface_8hpp.html</filename>
    <includes id="event_8hpp" name="event.hpp" local="yes" imported="no">rclcpp/event.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="qos_8hpp" name="qos.hpp" local="yes" imported="no">rclcpp/qos.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::TopicEndpointInfo</class>
    <class kind="class">rclcpp::node_interfaces::NodeGraphInterface</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
    <member kind="enumeration">
      <type></type>
      <name>EndpointType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ad47a956a91d1787241c564827be18aa5</anchor>
      <arglist></arglist>
      <enumvalue file="namespacerclcpp.html" anchor="ad47a956a91d1787241c564827be18aa5a4bbb8f967da6d1a610596d7257179c2b">Invalid</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="ad47a956a91d1787241c564827be18aa5a32c73be0cb175da278c8e2af0811b0d1">Publisher</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="ad47a956a91d1787241c564827be18aa5a787ad0b7a17de4ad6b1711bbf8d79fcb">Subscription</enumvalue>
    </member>
  </compound>
  <compound kind="file">
    <name>node_logging.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__logging_8hpp.html</filename>
    <includes id="logger_8hpp" name="logger.hpp" local="yes" imported="no">rclcpp/logger.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__logging__interface_8hpp" name="node_logging_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_logging_interface.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeLogging</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_logging_interface.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__logging__interface_8hpp.html</filename>
    <includes id="logger_8hpp" name="logger.hpp" local="yes" imported="no">rclcpp/logger.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeLoggingInterface</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_parameters.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__parameters_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__logging__interface_8hpp" name="node_logging_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_logging_interface.hpp</includes>
    <includes id="node__parameters__interface_8hpp" name="node_parameters_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_parameters_interface.hpp</includes>
    <includes id="node__services__interface_8hpp" name="node_services_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_services_interface.hpp</includes>
    <includes id="node__topics__interface_8hpp" name="node_topics_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_topics_interface.hpp</includes>
    <includes id="parameter_8hpp" name="parameter.hpp" local="yes" imported="no">rclcpp/parameter.hpp</includes>
    <includes id="parameter__service_8hpp" name="parameter_service.hpp" local="yes" imported="no">rclcpp/parameter_service.hpp</includes>
    <includes id="publisher_8hpp" name="publisher.hpp" local="yes" imported="no">rclcpp/publisher.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="struct">rclcpp::node_interfaces::ParameterInfo</class>
    <class kind="class">rclcpp::node_interfaces::ParameterMutationRecursionGuard</class>
    <class kind="class">rclcpp::node_interfaces::NodeParameters</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_parameters_interface.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__parameters__interface_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="parameter_8hpp" name="parameter.hpp" local="yes" imported="no">rclcpp/parameter.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="struct">rclcpp::node_interfaces::OnSetParametersCallbackHandle</class>
    <class kind="class">rclcpp::node_interfaces::NodeParametersInterface</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_services.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__services_8hpp.html</filename>
    <includes id="callback__group_8hpp" name="callback_group.hpp" local="yes" imported="no">rclcpp/callback_group.hpp</includes>
    <includes id="client_8hpp" name="client.hpp" local="yes" imported="no">rclcpp/client.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__services__interface_8hpp" name="node_services_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_services_interface.hpp</includes>
    <includes id="service_8hpp" name="service.hpp" local="yes" imported="no">rclcpp/service.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeServices</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_services_interface.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__services__interface_8hpp.html</filename>
    <includes id="callback__group_8hpp" name="callback_group.hpp" local="yes" imported="no">rclcpp/callback_group.hpp</includes>
    <includes id="client_8hpp" name="client.hpp" local="yes" imported="no">rclcpp/client.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="service_8hpp" name="service.hpp" local="yes" imported="no">rclcpp/service.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeServicesInterface</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_time_source.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__time__source_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__clock__interface_8hpp" name="node_clock_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_clock_interface.hpp</includes>
    <includes id="node__graph__interface_8hpp" name="node_graph_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_graph_interface.hpp</includes>
    <includes id="node__logging__interface_8hpp" name="node_logging_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_logging_interface.hpp</includes>
    <includes id="node__parameters__interface_8hpp" name="node_parameters_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_parameters_interface.hpp</includes>
    <includes id="node__services__interface_8hpp" name="node_services_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_services_interface.hpp</includes>
    <includes id="node__time__source__interface_8hpp" name="node_time_source_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_time_source_interface.hpp</includes>
    <includes id="node__topics__interface_8hpp" name="node_topics_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_topics_interface.hpp</includes>
    <includes id="time__source_8hpp" name="time_source.hpp" local="yes" imported="no">rclcpp/time_source.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeTimeSource</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_time_source_interface.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__time__source__interface_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeTimeSourceInterface</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_timers.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__timers_8hpp.html</filename>
    <includes id="callback__group_8hpp" name="callback_group.hpp" local="yes" imported="no">rclcpp/callback_group.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__timers__interface_8hpp" name="node_timers_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_timers_interface.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeTimers</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_timers_interface.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__timers__interface_8hpp.html</filename>
    <includes id="callback__group_8hpp" name="callback_group.hpp" local="yes" imported="no">rclcpp/callback_group.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeTimersInterface</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_topics.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__topics_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__timers__interface_8hpp" name="node_timers_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_timers_interface.hpp</includes>
    <includes id="node__topics__interface_8hpp" name="node_topics_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_topics_interface.hpp</includes>
    <includes id="publisher_8hpp" name="publisher.hpp" local="yes" imported="no">rclcpp/publisher.hpp</includes>
    <includes id="publisher__factory_8hpp" name="publisher_factory.hpp" local="yes" imported="no">rclcpp/publisher_factory.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeTopics</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_topics_interface.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__topics__interface_8hpp.html</filename>
    <includes id="callback__group_8hpp" name="callback_group.hpp" local="yes" imported="no">rclcpp/callback_group.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__timers__interface_8hpp" name="node_timers_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_timers_interface.hpp</includes>
    <includes id="publisher_8hpp" name="publisher.hpp" local="yes" imported="no">rclcpp/publisher.hpp</includes>
    <includes id="publisher__factory_8hpp" name="publisher_factory.hpp" local="yes" imported="no">rclcpp/publisher_factory.hpp</includes>
    <includes id="subscription_8hpp" name="subscription.hpp" local="yes" imported="no">rclcpp/subscription.hpp</includes>
    <includes id="subscription__factory_8hpp" name="subscription_factory.hpp" local="yes" imported="no">rclcpp/subscription_factory.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeTopicsInterface</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_waitables.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__waitables_8hpp.html</filename>
    <includes id="callback__group_8hpp" name="callback_group.hpp" local="yes" imported="no">rclcpp/callback_group.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="node__waitables__interface_8hpp" name="node_waitables_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_waitables_interface.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeWaitables</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_waitables_interface.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/node_interfaces/</path>
    <filename>node__waitables__interface_8hpp.html</filename>
    <includes id="callback__group_8hpp" name="callback_group.hpp" local="yes" imported="no">rclcpp/callback_group.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::node_interfaces::NodeWaitablesInterface</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>node_options.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>node__options_8hpp.html</filename>
    <includes id="context_8hpp" name="context.hpp" local="yes" imported="no">rclcpp/context.hpp</includes>
    <includes id="default__context_8hpp" name="default_context.hpp" local="yes" imported="no">rclcpp/contexts/default_context.hpp</includes>
    <includes id="parameter_8hpp" name="parameter.hpp" local="yes" imported="no">rclcpp/parameter.hpp</includes>
    <includes id="publisher__options_8hpp" name="publisher_options.hpp" local="yes" imported="no">rclcpp/publisher_options.hpp</includes>
    <includes id="qos_8hpp" name="qos.hpp" local="yes" imported="no">rclcpp/qos.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::NodeOptions</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>parameter.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>parameter_8hpp.html</filename>
    <includes id="exceptions_8hpp" name="exceptions.hpp" local="yes" imported="no">rclcpp/exceptions.hpp</includes>
    <includes id="parameter__value_8hpp" name="parameter_value.hpp" local="yes" imported="no">rclcpp/parameter_value.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::Parameter</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
    <namespace>rclcpp::detail</namespace>
    <namespace>std</namespace>
    <member kind="function">
      <type>auto</type>
      <name>get_value_helper</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>ae4f35ac4285bf465bb224eacbe09d2dc</anchor>
      <arglist>(const rclcpp::Parameter *parameter)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>_to_json_dict_entry</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a4d8f47054b6c550888eca755f3203f05</anchor>
      <arglist>(const Parameter &amp;param)</arglist>
    </member>
    <member kind="function">
      <type>std::ostream &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a6980e39627cfc70b542040ada42ca607</anchor>
      <arglist>(std::ostream &amp;os, const rclcpp::Parameter &amp;pv)</arglist>
    </member>
    <member kind="function">
      <type>std::ostream &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a0e9c5f2bb49142b5f5ad11a8cf1c9328</anchor>
      <arglist>(std::ostream &amp;os, const std::vector&lt; Parameter &gt; &amp;parameters)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>get_value_helper&lt; rclcpp::ParameterValue &gt;</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>a014059cadd00427bd36426b2a768566e</anchor>
      <arglist>(const rclcpp::Parameter *parameter)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>get_value_helper&lt; rclcpp::Parameter &gt;</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>af30f36a40ef365a465b6efc25e9eb7cf</anchor>
      <arglist>(const rclcpp::Parameter *parameter)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>namespacestd.html</anchorfile>
      <anchor>a1d2324ac3fe483483ba53560318b8639</anchor>
      <arglist>(const rclcpp::Parameter &amp;param)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>namespacestd.html</anchorfile>
      <anchor>ad5ab824f9738a59b56721b0a1517bda9</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameters)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>parameter_client.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>parameter__client_8hpp.html</filename>
    <includes id="executors_8hpp" name="executors.hpp" local="yes" imported="no">rclcpp/executors.hpp</includes>
    <includes id="create__subscription_8hpp" name="create_subscription.hpp" local="yes" imported="no">rclcpp/create_subscription.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node_8hpp" name="node.hpp" local="yes" imported="no">rclcpp/node.hpp</includes>
    <includes id="parameter_8hpp" name="parameter.hpp" local="yes" imported="no">rclcpp/parameter.hpp</includes>
    <includes id="type__support__decl_8hpp" name="type_support_decl.hpp" local="yes" imported="no">rclcpp/type_support_decl.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::AsyncParametersClient</class>
    <class kind="class">rclcpp::SyncParametersClient</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>parameter_events_filter.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>parameter__events__filter_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="parameter_8hpp" name="parameter.hpp" local="yes" imported="no">rclcpp/parameter.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::ParameterEventsFilter</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>parameter_map.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>parameter__map_8hpp.html</filename>
    <includes id="exceptions_8hpp" name="exceptions.hpp" local="yes" imported="no">rclcpp/exceptions.hpp</includes>
    <includes id="parameter_8hpp" name="parameter.hpp" local="yes" imported="no">rclcpp/parameter.hpp</includes>
    <includes id="parameter__value_8hpp" name="parameter_value.hpp" local="yes" imported="no">rclcpp/parameter_value.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <namespace>rclcpp</namespace>
    <member kind="typedef">
      <type>std::unordered_map&lt; std::string, std::vector&lt; Parameter &gt; &gt;</type>
      <name>ParameterMap</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa587f11d0e53c713ccc0addf5132d46a</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>ParameterMap</type>
      <name>parameter_map_from</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa974dc62646d9123ab206ca602c5e089</anchor>
      <arglist>(const rcl_params_t *const c_params)</arglist>
    </member>
    <member kind="function">
      <type>ParameterValue</type>
      <name>parameter_value_from</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa1a3f3243d1e335570334169979080cd</anchor>
      <arglist>(const rcl_variant_t *const c_value)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>parameter_service.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>parameter__service_8hpp.html</filename>
    <includes id="executors_8hpp" name="executors.hpp" local="yes" imported="no">rclcpp/executors.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node_8hpp" name="node.hpp" local="yes" imported="no">rclcpp/node.hpp</includes>
    <includes id="parameter_8hpp" name="parameter.hpp" local="yes" imported="no">rclcpp/parameter.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::ParameterService</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>parameter_value.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>parameter__value_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::ParameterTypeException</class>
    <class kind="class">rclcpp::ParameterValue</class>
    <namespace>rclcpp</namespace>
    <member kind="enumeration">
      <type></type>
      <name>ParameterType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_NOT_SET</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784adefddb470d1054265245a0c5dbeccf4e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_BOOL</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784a525d4dd60c307e7fb76d57cdeafce8b1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_INTEGER</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784a3a70ea9e9528656834106cf8a647e878</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_DOUBLE</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784a21dd7d8db14cc732692bea98eb073c20</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_STRING</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784a1ed2b44e2043e3cf925a1891e19fd73a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_BYTE_ARRAY</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784aac6f099e5c9b63fd51a75ae587a98271</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_BOOL_ARRAY</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784ab14cef41f8d8d14f9beef95eeb6dd91d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_INTEGER_ARRAY</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784a5b5c50ef3c04a90a59a31b3ad20a4495</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_DOUBLE_ARRAY</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784af685d20a1a39f3531732919ee99f44ab</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_STRING_ARRAY</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784ab19e036eb99725754e11c35c6f8d9eb1</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a36b93cd43e33e496e2a43f3eb9504b12</anchor>
      <arglist>(ParameterType type)</arglist>
    </member>
    <member kind="function">
      <type>std::ostream &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aed5caea480fefe503c1a7a02b4024dca</anchor>
      <arglist>(std::ostream &amp;os, ParameterType type)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>acc7cfb4c8905865fb02f3c8018657057</anchor>
      <arglist>(const ParameterValue &amp;type)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>publisher.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>publisher_8hpp.html</filename>
    <includes id="allocator__common_8hpp" name="allocator_common.hpp" local="yes" imported="no">rclcpp/allocator/allocator_common.hpp</includes>
    <includes id="allocator__deleter_8hpp" name="allocator_deleter.hpp" local="yes" imported="no">rclcpp/allocator/allocator_deleter.hpp</includes>
    <includes id="resolve__use__intra__process_8hpp" name="resolve_use_intra_process.hpp" local="yes" imported="no">rclcpp/detail/resolve_use_intra_process.hpp</includes>
    <includes id="intra__process__manager_8hpp" name="intra_process_manager.hpp" local="yes" imported="no">rclcpp/experimental/intra_process_manager.hpp</includes>
    <includes id="loaned__message_8hpp" name="loaned_message.hpp" local="yes" imported="no">rclcpp/loaned_message.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="publisher__base_8hpp" name="publisher_base.hpp" local="yes" imported="no">rclcpp/publisher_base.hpp</includes>
    <includes id="publisher__options_8hpp" name="publisher_options.hpp" local="yes" imported="no">rclcpp/publisher_options.hpp</includes>
    <includes id="type__support__decl_8hpp" name="type_support_decl.hpp" local="yes" imported="no">rclcpp/type_support_decl.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::LoanedMessage</class>
    <class kind="class">rclcpp::Publisher</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>publisher_base.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>publisher__base_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="qos_8hpp" name="qos.hpp" local="yes" imported="no">rclcpp/qos.hpp</includes>
    <includes id="qos__event_8hpp" name="qos_event.hpp" local="yes" imported="no">rclcpp/qos_event.hpp</includes>
    <includes id="type__support__decl_8hpp" name="type_support_decl.hpp" local="yes" imported="no">rclcpp/type_support_decl.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::PublisherBase</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
    <namespace>rclcpp::experimental</namespace>
  </compound>
  <compound kind="file">
    <name>publisher_factory.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>publisher__factory_8hpp.html</filename>
    <includes id="publisher_8hpp" name="publisher.hpp" local="yes" imported="no">rclcpp/publisher.hpp</includes>
    <includes id="publisher__base_8hpp" name="publisher_base.hpp" local="yes" imported="no">rclcpp/publisher_base.hpp</includes>
    <includes id="publisher__options_8hpp" name="publisher_options.hpp" local="yes" imported="no">rclcpp/publisher_options.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="qos_8hpp" name="qos.hpp" local="yes" imported="no">rclcpp/qos.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="struct">rclcpp::PublisherFactory</class>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>PublisherFactory</type>
      <name>create_publisher_factory</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a16897dedbda576cf6035188bdc7365d7</anchor>
      <arglist>(const rclcpp::PublisherOptionsWithAllocator&lt; AllocatorT &gt; &amp;options)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>publisher_options.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>publisher__options_8hpp.html</filename>
    <includes id="allocator__common_8hpp" name="allocator_common.hpp" local="yes" imported="no">rclcpp/allocator/allocator_common.hpp</includes>
    <includes id="rmw__implementation__specific__publisher__payload_8hpp" name="rmw_implementation_specific_publisher_payload.hpp" local="yes" imported="no">rclcpp/detail/rmw_implementation_specific_publisher_payload.hpp</includes>
    <includes id="intra__process__setting_8hpp" name="intra_process_setting.hpp" local="yes" imported="no">rclcpp/intra_process_setting.hpp</includes>
    <includes id="qos_8hpp" name="qos.hpp" local="yes" imported="no">rclcpp/qos.hpp</includes>
    <includes id="qos__event_8hpp" name="qos_event.hpp" local="yes" imported="no">rclcpp/qos_event.hpp</includes>
    <class kind="struct">rclcpp::PublisherOptionsBase</class>
    <class kind="struct">rclcpp::PublisherOptionsWithAllocator</class>
    <namespace>rclcpp</namespace>
    <member kind="typedef">
      <type>PublisherOptionsWithAllocator&lt; std::allocator&lt; void &gt; &gt;</type>
      <name>PublisherOptions</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a9f38e986e5843aa39fc2b9fff9cf927f</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>qos.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>qos_8hpp.html</filename>
    <includes id="duration_8hpp" name="duration.hpp" local="yes" imported="no">rclcpp/duration.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="struct">rclcpp::QoSInitialization</class>
    <class kind="struct">rclcpp::KeepAll</class>
    <class kind="struct">rclcpp::KeepLast</class>
    <class kind="class">rclcpp::QoS</class>
    <class kind="class">rclcpp::SensorDataQoS</class>
    <class kind="class">rclcpp::ParametersQoS</class>
    <class kind="class">rclcpp::ServicesQoS</class>
    <class kind="class">rclcpp::ParameterEventsQoS</class>
    <class kind="class">rclcpp::SystemDefaultsQoS</class>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>std::string</type>
      <name>qos_policy_name_from_kind</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a7bf3573ad178cad6dcdad0e8cdbdfe6f</anchor>
      <arglist>(rmw_qos_policy_kind_t policy_kind)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa9c7e823d8b8cbde96e0e295065c3836</anchor>
      <arglist>(const QoS &amp;left, const QoS &amp;right)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>af233f254408ae99037eaef31e6655e0d</anchor>
      <arglist>(const QoS &amp;left, const QoS &amp;right)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>qos_event.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>qos__event_8hpp.html</filename>
    <includes id="exceptions_8hpp" name="exceptions.hpp" local="yes" imported="no">rclcpp/exceptions.hpp</includes>
    <includes id="function__traits_8hpp" name="function_traits.hpp" local="yes" imported="no">rclcpp/function_traits.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="struct">rclcpp::PublisherEventCallbacks</class>
    <class kind="struct">rclcpp::SubscriptionEventCallbacks</class>
    <class kind="class">rclcpp::UnsupportedEventTypeException</class>
    <class kind="class">rclcpp::QOSEventHandlerBase</class>
    <class kind="class">rclcpp::QOSEventHandler</class>
    <namespace>rclcpp</namespace>
    <member kind="typedef">
      <type>rmw_requested_deadline_missed_status_t</type>
      <name>QOSDeadlineRequestedInfo</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a51f406f0e8e4928bd5490d5b170c2c3b</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rmw_offered_deadline_missed_status_t</type>
      <name>QOSDeadlineOfferedInfo</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aedf85a88396716dfeff77447176ed05c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rmw_liveliness_changed_status_t</type>
      <name>QOSLivelinessChangedInfo</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a144fad513d523f56d77a071740da8d11</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rmw_liveliness_lost_status_t</type>
      <name>QOSLivelinessLostInfo</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a0a89114d63f459ecc69241fa4f0dfc1f</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rmw_offered_qos_incompatible_event_status_t</type>
      <name>QOSOfferedIncompatibleQoSInfo</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>af6174538b0f569e38ba3be09812ad54c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rmw_requested_qos_incompatible_event_status_t</type>
      <name>QOSRequestedIncompatibleQoSInfo</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a26c69f1ce0937b36bc3915e989fb9faf</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(QOSDeadlineRequestedInfo &amp;)&gt;</type>
      <name>QOSDeadlineRequestedCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a742c7a1550bb04d91fc14f92adffd218</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(QOSDeadlineOfferedInfo &amp;)&gt;</type>
      <name>QOSDeadlineOfferedCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>abb6987cb949f1c2992beb9887311a5b6</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(QOSLivelinessChangedInfo &amp;)&gt;</type>
      <name>QOSLivelinessChangedCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a51148cfeddce96db7ad911fa893a4d82</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(QOSLivelinessLostInfo &amp;)&gt;</type>
      <name>QOSLivelinessLostCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a33673fe9ebe74818bc56d517eae84a03</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(QOSOfferedIncompatibleQoSInfo &amp;)&gt;</type>
      <name>QOSOfferedIncompatibleQoSCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a859176cdce7e529ada6dfa9b0c41be24</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(QOSRequestedIncompatibleQoSInfo &amp;)&gt;</type>
      <name>QOSRequestedIncompatibleQoSCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ab7ae5f255085a1ee084a83b7b8ec4ddb</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>rate.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>rate_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="utilities_8hpp" name="utilities.hpp" local="yes" imported="no">rclcpp/utilities.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::RateBase</class>
    <class kind="class">rclcpp::GenericRate</class>
    <namespace>rclcpp</namespace>
    <member kind="typedef">
      <type>GenericRate&lt; std::chrono::system_clock &gt;</type>
      <name>Rate</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>abe6d4a275a0f75ba817c8486f641f4ea</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>GenericRate&lt; std::chrono::steady_clock &gt;</type>
      <name>WallRate</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a9c57364eb16ca720485d170ee5b99cf7</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>rclcpp.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>rclcpp_8hpp.html</filename>
    <includes id="executors_8hpp" name="executors.hpp" local="yes" imported="no">rclcpp/executors.hpp</includes>
    <includes id="guard__condition_8hpp" name="guard_condition.hpp" local="yes" imported="no">rclcpp/guard_condition.hpp</includes>
    <includes id="node_8hpp" name="node.hpp" local="yes" imported="no">rclcpp/node.hpp</includes>
    <includes id="parameter_8hpp" name="parameter.hpp" local="yes" imported="no">rclcpp/parameter.hpp</includes>
    <includes id="parameter__client_8hpp" name="parameter_client.hpp" local="yes" imported="no">rclcpp/parameter_client.hpp</includes>
    <includes id="parameter__service_8hpp" name="parameter_service.hpp" local="yes" imported="no">rclcpp/parameter_service.hpp</includes>
    <includes id="rate_8hpp" name="rate.hpp" local="yes" imported="no">rclcpp/rate.hpp</includes>
    <includes id="time_8hpp" name="time.hpp" local="yes" imported="no">rclcpp/time.hpp</includes>
    <includes id="utilities_8hpp" name="utilities.hpp" local="yes" imported="no">rclcpp/utilities.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="wait__set_8hpp" name="wait_set.hpp" local="yes" imported="no">rclcpp/wait_set.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
  </compound>
  <compound kind="file">
    <name>scope_exit.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>scope__exit_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <class kind="struct">rclcpp::ScopeExit</class>
    <namespace>rclcpp</namespace>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_SCOPE_EXIT</name>
      <anchorfile>scope__exit_8hpp.html</anchorfile>
      <anchor>a86882d99f5485ef666db981e86b8df37</anchor>
      <arglist>(code)</arglist>
    </member>
    <member kind="function">
      <type>ScopeExit&lt; Callable &gt;</type>
      <name>make_scope_exit</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>afc7846ba2bee783fa9352c7073f30eec</anchor>
      <arglist>(Callable callable)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>serialization.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>serialization_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="struct">rclcpp::serialization_traits::is_serialized_message_class</class>
    <class kind="struct">rclcpp::serialization_traits::is_serialized_message_class&lt; SerializedMessage &gt;</class>
    <class kind="class">rclcpp::SerializationBase</class>
    <class kind="class">rclcpp::Serialization</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::serialization_traits</namespace>
  </compound>
  <compound kind="file">
    <name>serialized_message.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>serialized__message_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::SerializedMessage</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>service.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>service_8hpp.html</filename>
    <includes id="any__service__callback_8hpp" name="any_service_callback.hpp" local="yes" imported="no">rclcpp/any_service_callback.hpp</includes>
    <includes id="exceptions_8hpp" name="exceptions.hpp" local="yes" imported="no">rclcpp/exceptions.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="type__support__decl_8hpp" name="type_support_decl.hpp" local="yes" imported="no">rclcpp/type_support_decl.hpp</includes>
    <includes id="expand__topic__or__service__name_8hpp" name="expand_topic_or_service_name.hpp" local="yes" imported="no">rclcpp/expand_topic_or_service_name.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::ServiceBase</class>
    <class kind="class">rclcpp::Service</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>allocator_memory_strategy.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/strategies/</path>
    <filename>allocator__memory__strategy_8hpp.html</filename>
    <includes id="allocator__common_8hpp" name="allocator_common.hpp" local="yes" imported="no">rclcpp/allocator/allocator_common.hpp</includes>
    <includes id="memory__strategy_8hpp" name="memory_strategy.hpp" local="yes" imported="no">rclcpp/memory_strategy.hpp</includes>
    <includes id="node_8hpp" name="node.hpp" local="yes" imported="no">rclcpp/node.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::memory_strategies</namespace>
    <namespace>rclcpp::memory_strategies::allocator_memory_strategy</namespace>
  </compound>
  <compound kind="file">
    <name>message_pool_memory_strategy.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/strategies/</path>
    <filename>message__pool__memory__strategy_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="message__memory__strategy_8hpp" name="message_memory_strategy.hpp" local="yes" imported="no">rclcpp/message_memory_strategy.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy</class>
    <class kind="struct">rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy::PoolMember</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::strategies</namespace>
    <namespace>rclcpp::strategies::message_pool_memory_strategy</namespace>
  </compound>
  <compound kind="file">
    <name>subscription.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>subscription_8hpp.html</filename>
    <includes id="any__subscription__callback_8hpp" name="any_subscription_callback.hpp" local="yes" imported="no">rclcpp/any_subscription_callback.hpp</includes>
    <includes id="resolve__use__intra__process_8hpp" name="resolve_use_intra_process.hpp" local="yes" imported="no">rclcpp/detail/resolve_use_intra_process.hpp</includes>
    <includes id="resolve__intra__process__buffer__type_8hpp" name="resolve_intra_process_buffer_type.hpp" local="yes" imported="no">rclcpp/detail/resolve_intra_process_buffer_type.hpp</includes>
    <includes id="exceptions_8hpp" name="exceptions.hpp" local="yes" imported="no">rclcpp/exceptions.hpp</includes>
    <includes id="expand__topic__or__service__name_8hpp" name="expand_topic_or_service_name.hpp" local="yes" imported="no">rclcpp/expand_topic_or_service_name.hpp</includes>
    <includes id="intra__process__manager_8hpp" name="intra_process_manager.hpp" local="yes" imported="no">rclcpp/experimental/intra_process_manager.hpp</includes>
    <includes id="subscription__intra__process_8hpp" name="subscription_intra_process.hpp" local="yes" imported="no">rclcpp/experimental/subscription_intra_process.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="message__info_8hpp" name="message_info.hpp" local="yes" imported="no">rclcpp/message_info.hpp</includes>
    <includes id="message__memory__strategy_8hpp" name="message_memory_strategy.hpp" local="yes" imported="no">rclcpp/message_memory_strategy.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="subscription__base_8hpp" name="subscription_base.hpp" local="yes" imported="no">rclcpp/subscription_base.hpp</includes>
    <includes id="subscription__options_8hpp" name="subscription_options.hpp" local="yes" imported="no">rclcpp/subscription_options.hpp</includes>
    <includes id="subscription__traits_8hpp" name="subscription_traits.hpp" local="yes" imported="no">rclcpp/subscription_traits.hpp</includes>
    <includes id="type__support__decl_8hpp" name="type_support_decl.hpp" local="yes" imported="no">rclcpp/type_support_decl.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <includes id="subscription__topic__statistics_8hpp" name="subscription_topic_statistics.hpp" local="yes" imported="no">rclcpp/topic_statistics/subscription_topic_statistics.hpp</includes>
    <class kind="class">rclcpp::Subscription</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
  </compound>
  <compound kind="file">
    <name>subscription_base.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>subscription__base_8hpp.html</filename>
    <includes id="any__subscription__callback_8hpp" name="any_subscription_callback.hpp" local="yes" imported="no">rclcpp/any_subscription_callback.hpp</includes>
    <includes id="intra__process__manager_8hpp" name="intra_process_manager.hpp" local="yes" imported="no">rclcpp/experimental/intra_process_manager.hpp</includes>
    <includes id="subscription__intra__process__base_8hpp" name="subscription_intra_process_base.hpp" local="yes" imported="no">rclcpp/experimental/subscription_intra_process_base.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="message__info_8hpp" name="message_info.hpp" local="yes" imported="no">rclcpp/message_info.hpp</includes>
    <includes id="qos_8hpp" name="qos.hpp" local="yes" imported="no">rclcpp/qos.hpp</includes>
    <includes id="qos__event_8hpp" name="qos_event.hpp" local="yes" imported="no">rclcpp/qos_event.hpp</includes>
    <includes id="serialized__message_8hpp" name="serialized_message.hpp" local="yes" imported="no">rclcpp/serialized_message.hpp</includes>
    <includes id="type__support__decl_8hpp" name="type_support_decl.hpp" local="yes" imported="no">rclcpp/type_support_decl.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::SubscriptionBase</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
    <namespace>rclcpp::experimental</namespace>
  </compound>
  <compound kind="file">
    <name>subscription_factory.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>subscription__factory_8hpp.html</filename>
    <includes id="any__subscription__callback_8hpp" name="any_subscription_callback.hpp" local="yes" imported="no">rclcpp/any_subscription_callback.hpp</includes>
    <includes id="intra__process__buffer__type_8hpp" name="intra_process_buffer_type.hpp" local="yes" imported="no">rclcpp/intra_process_buffer_type.hpp</includes>
    <includes id="node__base__interface_8hpp" name="node_base_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_base_interface.hpp</includes>
    <includes id="qos_8hpp" name="qos.hpp" local="yes" imported="no">rclcpp/qos.hpp</includes>
    <includes id="subscription_8hpp" name="subscription.hpp" local="yes" imported="no">rclcpp/subscription.hpp</includes>
    <includes id="subscription__options_8hpp" name="subscription_options.hpp" local="yes" imported="no">rclcpp/subscription_options.hpp</includes>
    <includes id="subscription__traits_8hpp" name="subscription_traits.hpp" local="yes" imported="no">rclcpp/subscription_traits.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="subscription__topic__statistics_8hpp" name="subscription_topic_statistics.hpp" local="yes" imported="no">rclcpp/topic_statistics/subscription_topic_statistics.hpp</includes>
    <class kind="struct">rclcpp::SubscriptionFactory</class>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>SubscriptionFactory</type>
      <name>create_subscription_factory</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a3de16fd4688e653846d71dbd40f79b09</anchor>
      <arglist>(CallbackT &amp;&amp;callback, const rclcpp::SubscriptionOptionsWithAllocator&lt; AllocatorT &gt; &amp;options, typename MessageMemoryStrategyT::SharedPtr msg_mem_strat, std::shared_ptr&lt; rclcpp::topic_statistics::SubscriptionTopicStatistics&lt; CallbackMessageT &gt;&gt; subscription_topic_stats=nullptr)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>subscription_options.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>subscription__options_8hpp.html</filename>
    <includes id="callback__group_8hpp" name="callback_group.hpp" local="yes" imported="no">rclcpp/callback_group.hpp</includes>
    <includes id="rmw__implementation__specific__subscription__payload_8hpp" name="rmw_implementation_specific_subscription_payload.hpp" local="yes" imported="no">rclcpp/detail/rmw_implementation_specific_subscription_payload.hpp</includes>
    <includes id="intra__process__buffer__type_8hpp" name="intra_process_buffer_type.hpp" local="yes" imported="no">rclcpp/intra_process_buffer_type.hpp</includes>
    <includes id="intra__process__setting_8hpp" name="intra_process_setting.hpp" local="yes" imported="no">rclcpp/intra_process_setting.hpp</includes>
    <includes id="qos_8hpp" name="qos.hpp" local="yes" imported="no">rclcpp/qos.hpp</includes>
    <includes id="qos__event_8hpp" name="qos_event.hpp" local="yes" imported="no">rclcpp/qos_event.hpp</includes>
    <includes id="topic__statistics__state_8hpp" name="topic_statistics_state.hpp" local="yes" imported="no">rclcpp/topic_statistics_state.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="struct">rclcpp::SubscriptionOptionsBase</class>
    <class kind="struct">rclcpp::SubscriptionOptionsBase::TopicStatisticsOptions</class>
    <class kind="struct">rclcpp::SubscriptionOptionsWithAllocator</class>
    <namespace>rclcpp</namespace>
    <member kind="typedef">
      <type>SubscriptionOptionsWithAllocator&lt; std::allocator&lt; void &gt; &gt;</type>
      <name>SubscriptionOptions</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa0ce420d67bb40b61156347455d3ad23</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>subscription_traits.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>subscription__traits_8hpp.html</filename>
    <includes id="function__traits_8hpp" name="function_traits.hpp" local="yes" imported="no">rclcpp/function_traits.hpp</includes>
    <includes id="serialized__message_8hpp" name="serialized_message.hpp" local="yes" imported="no">rclcpp/serialized_message.hpp</includes>
    <includes id="subscription__options_8hpp" name="subscription_options.hpp" local="yes" imported="no">rclcpp/subscription_options.hpp</includes>
    <class kind="struct">rclcpp::subscription_traits::is_serialized_subscription_argument</class>
    <class kind="struct">rclcpp::subscription_traits::is_serialized_subscription_argument&lt; SerializedMessage &gt;</class>
    <class kind="struct">rclcpp::subscription_traits::is_serialized_subscription_argument&lt; std::shared_ptr&lt; SerializedMessage &gt; &gt;</class>
    <class kind="struct">rclcpp::subscription_traits::is_serialized_subscription</class>
    <class kind="struct">rclcpp::subscription_traits::is_serialized_callback</class>
    <class kind="struct">rclcpp::subscription_traits::extract_message_type</class>
    <class kind="struct">rclcpp::subscription_traits::extract_message_type&lt; std::shared_ptr&lt; MessageT &gt; &gt;</class>
    <class kind="struct">rclcpp::subscription_traits::extract_message_type&lt; std::unique_ptr&lt; MessageT, Deleter &gt; &gt;</class>
    <class kind="struct">rclcpp::subscription_traits::has_message_type</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::subscription_traits</namespace>
  </compound>
  <compound kind="file">
    <name>subscription_wait_set_mask.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>subscription__wait__set__mask_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::SubscriptionWaitSetMask</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>time.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>time_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="duration_8hpp" name="duration.hpp" local="yes" imported="no">rclcpp/duration.hpp</includes>
    <class kind="class">rclcpp::Time</class>
    <namespace>rclcpp</namespace>
    <member kind="function">
      <type>Time</type>
      <name>operator+</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a31b2ba48a94966a93b36de2e52cae2bb</anchor>
      <arglist>(const rclcpp::Duration &amp;lhs, const rclcpp::Time &amp;rhs)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>time_source.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>time__source_8hpp.html</filename>
    <includes id="node_8hpp" name="node.hpp" local="yes" imported="no">rclcpp/node.hpp</includes>
    <includes id="node__parameters__interface_8hpp" name="node_parameters_interface.hpp" local="yes" imported="no">rclcpp/node_interfaces/node_parameters_interface.hpp</includes>
    <class kind="class">rclcpp::TimeSource</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>timer.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>timer_8hpp.html</filename>
    <includes id="clock_8hpp" name="clock.hpp" local="yes" imported="no">rclcpp/clock.hpp</includes>
    <includes id="context_8hpp" name="context.hpp" local="yes" imported="no">rclcpp/context.hpp</includes>
    <includes id="function__traits_8hpp" name="function_traits.hpp" local="yes" imported="no">rclcpp/function_traits.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="rate_8hpp" name="rate.hpp" local="yes" imported="no">rclcpp/rate.hpp</includes>
    <includes id="utilities_8hpp" name="utilities.hpp" local="yes" imported="no">rclcpp/utilities.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::TimerBase</class>
    <class kind="class">rclcpp::GenericTimer</class>
    <class kind="class">rclcpp::WallTimer</class>
    <namespace>rclcpp</namespace>
    <member kind="typedef">
      <type>std::function&lt; void()&gt;</type>
      <name>VoidCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aba01c682b9c93b3ff844cc36ea80fda1</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(TimerBase &amp;)&gt;</type>
      <name>TimerCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa6bb5a91fd8ef745ce261338f803fc9e</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>subscription_topic_statistics.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/topic_statistics/</path>
    <filename>subscription__topic__statistics_8hpp.html</filename>
    <includes id="time_8hpp" name="time.hpp" local="yes" imported="no">rclcpp/time.hpp</includes>
    <includes id="publisher_8hpp" name="publisher.hpp" local="yes" imported="no">rclcpp/publisher.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <class kind="class">rclcpp::topic_statistics::SubscriptionTopicStatistics</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::topic_statistics</namespace>
    <member kind="variable">
      <type>constexpr const char</type>
      <name>kDefaultPublishTopicName</name>
      <anchorfile>namespacerclcpp_1_1topic__statistics.html</anchorfile>
      <anchor>a54fcc5dacb1b61b72f17213ea9b347ca</anchor>
      <arglist>[]</arglist>
    </member>
    <member kind="variable">
      <type>constexpr const std::chrono::milliseconds</type>
      <name>kDefaultPublishingPeriod</name>
      <anchorfile>namespacerclcpp_1_1topic__statistics.html</anchorfile>
      <anchor>a6566f900f8440cc00d8870fdb2917c33</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>topic_statistics_state.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>topic__statistics__state_8hpp.html</filename>
    <namespace>rclcpp</namespace>
    <member kind="enumeration">
      <type></type>
      <name>TopicStatisticsState</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1faa0b05040d3cf0f74c231b6800bffc</anchor>
      <arglist></arglist>
      <enumvalue file="namespacerclcpp.html" anchor="a1faa0b05040d3cf0f74c231b6800bffca2faec1f9f8cc7f8f40d521c4dd574f49">Enable</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a1faa0b05040d3cf0f74c231b6800bffcabcfaccebf745acfd5e75351095a5394a">Disable</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a1faa0b05040d3cf0f74c231b6800bffca5e7bd84eadd196f52e8320680fa1c7cf">NodeDefault</enumvalue>
    </member>
  </compound>
  <compound kind="file">
    <name>type_support_decl.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>type__support__decl_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::type_support</namespace>
    <member kind="function">
      <type>const rosidl_message_type_support_t *</type>
      <name>get_intra_process_message_msg_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>a66bd1e46c8223c97596e8c3def51103e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_message_type_support_t *</type>
      <name>get_parameter_event_msg_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>a3a33f79e0edf74a7a3c7bbb15fac4f30</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_message_type_support_t *</type>
      <name>get_set_parameters_result_msg_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>afe2a99fc964d8f98b0cca073b5aa104d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_message_type_support_t *</type>
      <name>get_parameter_descriptor_msg_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>a1e6abad99cddb01e195ae6cf43da259a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_message_type_support_t *</type>
      <name>get_list_parameters_result_msg_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>ab09e2f3d329ea12b8078cf752b3d15b8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_service_type_support_t *</type>
      <name>get_get_parameters_srv_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>ad46e551c5bb32eebbf49d6395ab2c498</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_service_type_support_t *</type>
      <name>get_get_parameter_types_srv_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>a1a7b50e715313a225925f03bc3808ae4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_service_type_support_t *</type>
      <name>get_set_parameters_srv_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>af043d28ff378d26b0d4e478900bb19d5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_service_type_support_t *</type>
      <name>get_list_parameters_srv_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>af9023d99b556bced4348bb9a09fe194e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_service_type_support_t *</type>
      <name>get_describe_parameters_srv_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>a6576186afeeb70d483a98be8573a911a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_service_type_support_t *</type>
      <name>get_set_parameters_atomically_srv_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>a2339e1ef611d5baeebf698412bdafaaa</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>visibility_control.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>visibility__control_8hpp.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_EXPORT</name>
      <anchorfile>visibility__control_8hpp.html</anchorfile>
      <anchor>a1834d9793de3cc75964976872d3bc4e7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_IMPORT</name>
      <anchorfile>visibility__control_8hpp.html</anchorfile>
      <anchor>adab3eda5733c1b1b2dbfd9f5862ce2b9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_PUBLIC</name>
      <anchorfile>visibility__control_8hpp.html</anchorfile>
      <anchor>a802feb4ddc87a0699f6339a4b9ab7f23</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_LOCAL</name>
      <anchorfile>visibility__control_8hpp.html</anchorfile>
      <anchor>a14d1e1dddec7ab9b385bb674e866bb97</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RCLCPP_PUBLIC_TYPE</name>
      <anchorfile>visibility__control_8hpp.html</anchorfile>
      <anchor>ae6a62ebf5d69c6853af09a85d4257592</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>wait_result.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>wait__result_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="wait__result__kind_8hpp" name="wait_result_kind.hpp" local="yes" imported="no">rclcpp/wait_result_kind.hpp</includes>
    <class kind="class">rclcpp::WaitResult</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>wait_result_kind.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>wait__result__kind_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <namespace>rclcpp</namespace>
    <member kind="enumeration">
      <type></type>
      <name>WaitResultKind</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aac834535a6eab608968bbb44d9773bb1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Ready</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aac834535a6eab608968bbb44d9773bb1ad46050bbb7cc1d8c1836bdb94bc428b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Timeout</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aac834535a6eab608968bbb44d9773bb1ab55663adb793759edd2082b5194f1cd3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Empty</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aac834535a6eab608968bbb44d9773bb1aaaec822ce6caf3162cbe3cc2cfa1cdc0</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>wait_set.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>wait__set_8hpp.html</filename>
    <includes id="guard__condition_8hpp" name="guard_condition.hpp" local="yes" imported="no">rclcpp/guard_condition.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="dynamic__storage_8hpp" name="dynamic_storage.hpp" local="yes" imported="no">rclcpp/wait_set_policies/dynamic_storage.hpp</includes>
    <includes id="sequential__synchronization_8hpp" name="sequential_synchronization.hpp" local="yes" imported="no">rclcpp/wait_set_policies/sequential_synchronization.hpp</includes>
    <includes id="static__storage_8hpp" name="static_storage.hpp" local="yes" imported="no">rclcpp/wait_set_policies/static_storage.hpp</includes>
    <includes id="thread__safe__synchronization_8hpp" name="thread_safe_synchronization.hpp" local="yes" imported="no">rclcpp/wait_set_policies/thread_safe_synchronization.hpp</includes>
    <includes id="wait__set__template_8hpp" name="wait_set_template.hpp" local="yes" imported="no">rclcpp/wait_set_template.hpp</includes>
    <namespace>rclcpp</namespace>
    <member kind="typedef">
      <type>rclcpp::WaitSetTemplate&lt; rclcpp::wait_set_policies::SequentialSynchronization, rclcpp::wait_set_policies::DynamicStorage &gt;</type>
      <name>WaitSet</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ad6fb19c154de27e92430309d2da25ac3</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rclcpp::WaitSetTemplate&lt; rclcpp::wait_set_policies::SequentialSynchronization, rclcpp::wait_set_policies::StaticStorage&lt; NumberOfSubscriptions, NumberOfGuardCondtions, NumberOfTimers, NumberOfClients, NumberOfServices, NumberOfWaitables &gt; &gt;</type>
      <name>StaticWaitSet</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>adb06acf4a5723b1445fa6ed4e8f73374</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rclcpp::WaitSetTemplate&lt; rclcpp::wait_set_policies::ThreadSafeSynchronization, rclcpp::wait_set_policies::DynamicStorage &gt;</type>
      <name>ThreadSafeWaitSet</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>acaec573e71549fd3078644e18e7f7127</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>storage_policy_common.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/wait_set_policies/detail/</path>
    <filename>storage__policy__common_8hpp.html</filename>
    <includes id="exceptions_8hpp" name="exceptions.hpp" local="yes" imported="no">rclcpp/exceptions.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::wait_set_policies::detail::StoragePolicyCommon</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::wait_set_policies</namespace>
    <namespace>rclcpp::wait_set_policies::detail</namespace>
  </compound>
  <compound kind="file">
    <name>synchronization_policy_common.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/wait_set_policies/detail/</path>
    <filename>synchronization__policy__common_8hpp.html</filename>
    <class kind="class">rclcpp::wait_set_policies::detail::SynchronizationPolicyCommon</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::wait_set_policies</namespace>
    <namespace>rclcpp::wait_set_policies::detail</namespace>
  </compound>
  <compound kind="file">
    <name>write_preferring_read_write_lock.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/wait_set_policies/detail/</path>
    <filename>write__preferring__read__write__lock_8hpp.html</filename>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock</class>
    <class kind="class">rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock::ReadMutex</class>
    <class kind="class">rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock::WriteMutex</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::wait_set_policies</namespace>
    <namespace>rclcpp::wait_set_policies::detail</namespace>
  </compound>
  <compound kind="file">
    <name>dynamic_storage.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/wait_set_policies/</path>
    <filename>dynamic__storage_8hpp.html</filename>
    <includes id="client_8hpp" name="client.hpp" local="yes" imported="no">rclcpp/client.hpp</includes>
    <includes id="guard__condition_8hpp" name="guard_condition.hpp" local="yes" imported="no">rclcpp/guard_condition.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="service_8hpp" name="service.hpp" local="yes" imported="no">rclcpp/service.hpp</includes>
    <includes id="subscription__base_8hpp" name="subscription_base.hpp" local="yes" imported="no">rclcpp/subscription_base.hpp</includes>
    <includes id="subscription__wait__set__mask_8hpp" name="subscription_wait_set_mask.hpp" local="yes" imported="no">rclcpp/subscription_wait_set_mask.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="storage__policy__common_8hpp" name="storage_policy_common.hpp" local="yes" imported="no">rclcpp/wait_set_policies/detail/storage_policy_common.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::wait_set_policies::DynamicStorage</class>
    <class kind="class">rclcpp::wait_set_policies::DynamicStorage::SubscriptionEntry</class>
    <class kind="class">rclcpp::wait_set_policies::DynamicStorage::WeakSubscriptionEntry</class>
    <class kind="class">rclcpp::wait_set_policies::DynamicStorage::WaitableEntry</class>
    <class kind="class">rclcpp::wait_set_policies::DynamicStorage::WeakWaitableEntry</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::wait_set_policies</namespace>
  </compound>
  <compound kind="file">
    <name>sequential_synchronization.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/wait_set_policies/</path>
    <filename>sequential__synchronization_8hpp.html</filename>
    <includes id="client_8hpp" name="client.hpp" local="yes" imported="no">rclcpp/client.hpp</includes>
    <includes id="exceptions_8hpp" name="exceptions.hpp" local="yes" imported="no">rclcpp/exceptions.hpp</includes>
    <includes id="guard__condition_8hpp" name="guard_condition.hpp" local="yes" imported="no">rclcpp/guard_condition.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="service_8hpp" name="service.hpp" local="yes" imported="no">rclcpp/service.hpp</includes>
    <includes id="subscription__base_8hpp" name="subscription_base.hpp" local="yes" imported="no">rclcpp/subscription_base.hpp</includes>
    <includes id="subscription__wait__set__mask_8hpp" name="subscription_wait_set_mask.hpp" local="yes" imported="no">rclcpp/subscription_wait_set_mask.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="wait__result_8hpp" name="wait_result.hpp" local="yes" imported="no">rclcpp/wait_result.hpp</includes>
    <includes id="wait__result__kind_8hpp" name="wait_result_kind.hpp" local="yes" imported="no">rclcpp/wait_result_kind.hpp</includes>
    <includes id="synchronization__policy__common_8hpp" name="synchronization_policy_common.hpp" local="yes" imported="no">rclcpp/wait_set_policies/detail/synchronization_policy_common.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::wait_set_policies::SequentialSynchronization</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::wait_set_policies</namespace>
  </compound>
  <compound kind="file">
    <name>static_storage.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/wait_set_policies/</path>
    <filename>static__storage_8hpp.html</filename>
    <includes id="client_8hpp" name="client.hpp" local="yes" imported="no">rclcpp/client.hpp</includes>
    <includes id="guard__condition_8hpp" name="guard_condition.hpp" local="yes" imported="no">rclcpp/guard_condition.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="service_8hpp" name="service.hpp" local="yes" imported="no">rclcpp/service.hpp</includes>
    <includes id="subscription__base_8hpp" name="subscription_base.hpp" local="yes" imported="no">rclcpp/subscription_base.hpp</includes>
    <includes id="subscription__wait__set__mask_8hpp" name="subscription_wait_set_mask.hpp" local="yes" imported="no">rclcpp/subscription_wait_set_mask.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="storage__policy__common_8hpp" name="storage_policy_common.hpp" local="yes" imported="no">rclcpp/wait_set_policies/detail/storage_policy_common.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::wait_set_policies::StaticStorage</class>
    <class kind="class">rclcpp::wait_set_policies::StaticStorage::SubscriptionEntry</class>
    <class kind="struct">rclcpp::wait_set_policies::StaticStorage::WaitableEntry</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::wait_set_policies</namespace>
  </compound>
  <compound kind="file">
    <name>thread_safe_synchronization.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/wait_set_policies/</path>
    <filename>thread__safe__synchronization_8hpp.html</filename>
    <includes id="client_8hpp" name="client.hpp" local="yes" imported="no">rclcpp/client.hpp</includes>
    <includes id="exceptions_8hpp" name="exceptions.hpp" local="yes" imported="no">rclcpp/exceptions.hpp</includes>
    <includes id="guard__condition_8hpp" name="guard_condition.hpp" local="yes" imported="no">rclcpp/guard_condition.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="service_8hpp" name="service.hpp" local="yes" imported="no">rclcpp/service.hpp</includes>
    <includes id="subscription__base_8hpp" name="subscription_base.hpp" local="yes" imported="no">rclcpp/subscription_base.hpp</includes>
    <includes id="subscription__wait__set__mask_8hpp" name="subscription_wait_set_mask.hpp" local="yes" imported="no">rclcpp/subscription_wait_set_mask.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="wait__result_8hpp" name="wait_result.hpp" local="yes" imported="no">rclcpp/wait_result.hpp</includes>
    <includes id="wait__result__kind_8hpp" name="wait_result_kind.hpp" local="yes" imported="no">rclcpp/wait_result_kind.hpp</includes>
    <includes id="synchronization__policy__common_8hpp" name="synchronization_policy_common.hpp" local="yes" imported="no">rclcpp/wait_set_policies/detail/synchronization_policy_common.hpp</includes>
    <includes id="write__preferring__read__write__lock_8hpp" name="write_preferring_read_write_lock.hpp" local="yes" imported="no">rclcpp/wait_set_policies/detail/write_preferring_read_write_lock.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::wait_set_policies::ThreadSafeSynchronization</class>
    <namespace>rclcpp</namespace>
    <namespace>rclcpp::wait_set_policies</namespace>
  </compound>
  <compound kind="file">
    <name>wait_set_template.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>wait__set__template_8hpp.html</filename>
    <includes id="client_8hpp" name="client.hpp" local="yes" imported="no">rclcpp/client.hpp</includes>
    <includes id="context_8hpp" name="context.hpp" local="yes" imported="no">rclcpp/context.hpp</includes>
    <includes id="default__context_8hpp" name="default_context.hpp" local="yes" imported="no">rclcpp/contexts/default_context.hpp</includes>
    <includes id="guard__condition_8hpp" name="guard_condition.hpp" local="yes" imported="no">rclcpp/guard_condition.hpp</includes>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="scope__exit_8hpp" name="scope_exit.hpp" local="yes" imported="no">rclcpp/scope_exit.hpp</includes>
    <includes id="service_8hpp" name="service.hpp" local="yes" imported="no">rclcpp/service.hpp</includes>
    <includes id="subscription__base_8hpp" name="subscription_base.hpp" local="yes" imported="no">rclcpp/subscription_base.hpp</includes>
    <includes id="subscription__wait__set__mask_8hpp" name="subscription_wait_set_mask.hpp" local="yes" imported="no">rclcpp/subscription_wait_set_mask.hpp</includes>
    <includes id="timer_8hpp" name="timer.hpp" local="yes" imported="no">rclcpp/timer.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <includes id="wait__result_8hpp" name="wait_result.hpp" local="yes" imported="no">rclcpp/wait_result.hpp</includes>
    <includes id="waitable_8hpp" name="waitable.hpp" local="yes" imported="no">rclcpp/waitable.hpp</includes>
    <class kind="class">rclcpp::WaitSetTemplate</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="file">
    <name>waitable.hpp</name>
    <path>/opt/ros-src/src/ros2/rclcpp/rclcpp/include/rclcpp/</path>
    <filename>waitable_8hpp.html</filename>
    <includes id="macros_8hpp" name="macros.hpp" local="yes" imported="no">rclcpp/macros.hpp</includes>
    <includes id="visibility__control_8hpp" name="visibility_control.hpp" local="yes" imported="no">rclcpp/visibility_control.hpp</includes>
    <class kind="class">rclcpp::Waitable</class>
    <namespace>rclcpp</namespace>
  </compound>
  <compound kind="class">
    <name>rclcpp::allocator::AllocatorDeleter</name>
    <filename>classrclcpp_1_1allocator_1_1AllocatorDeleter.html</filename>
    <templarg></templarg>
    <member kind="function">
      <type></type>
      <name>AllocatorDeleter</name>
      <anchorfile>classrclcpp_1_1allocator_1_1AllocatorDeleter.html</anchorfile>
      <anchor>a3f27761f2de1b6afa4d079b6c9568412</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AllocatorDeleter</name>
      <anchorfile>classrclcpp_1_1allocator_1_1AllocatorDeleter.html</anchorfile>
      <anchor>abfee0991ff296f1a4feec2e67ec8dd94</anchor>
      <arglist>(Allocator *a)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AllocatorDeleter</name>
      <anchorfile>classrclcpp_1_1allocator_1_1AllocatorDeleter.html</anchorfile>
      <anchor>afb0beb325868f0abc25849d9aabdd85d</anchor>
      <arglist>(const AllocatorDeleter&lt; T &gt; &amp;a)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>operator()</name>
      <anchorfile>classrclcpp_1_1allocator_1_1AllocatorDeleter.html</anchorfile>
      <anchor>aeff3ddaa126aef9160d290bcb5aa95ee</anchor>
      <arglist>(T *ptr)</arglist>
    </member>
    <member kind="function">
      <type>Allocator *</type>
      <name>get_allocator</name>
      <anchorfile>classrclcpp_1_1allocator_1_1AllocatorDeleter.html</anchorfile>
      <anchor>aee564593690c59bb03cddef484658d73</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_allocator</name>
      <anchorfile>classrclcpp_1_1allocator_1_1AllocatorDeleter.html</anchorfile>
      <anchor>a7b3f0b659ef903991c2aeec748e951d3</anchor>
      <arglist>(Allocator *alloc)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy</name>
    <filename>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</filename>
    <templarg></templarg>
    <base>rclcpp::memory_strategy::MemoryStrategy</base>
    <member kind="typedef">
      <type>typename allocator::AllocRebind&lt; void *, Alloc &gt;</type>
      <name>VoidAllocTraits</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a5d35276dd5cbdb9cef1f7504119879e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename VoidAllocTraits::allocator_type</type>
      <name>VoidAlloc</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>aecb9b437636999fd7a8927121ebe3c9e</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AllocatorMemoryStrategy</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a96922dd77689dacb08dc32332dfd5e0c</anchor>
      <arglist>(std::shared_ptr&lt; Alloc &gt; allocator)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AllocatorMemoryStrategy</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a014c5f8cd08ba878e506f05300e298de</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_guard_condition</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a9213233fdd06911952cccef83205d9ce</anchor>
      <arglist>(const rcl_guard_condition_t *guard_condition) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_guard_condition</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a8b65ef3c8df7f861d8e05ba659aa0097</anchor>
      <arglist>(const rcl_guard_condition_t *guard_condition) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>clear_handles</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>adaa11435535eb97d643ce6537e7d9683</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_null_handles</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a023efc821cdb6beeaa253eb25ce9d2ec</anchor>
      <arglist>(rcl_wait_set_t *wait_set) override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>collect_entities</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>ac66974b49193262f3895f83e8351b12e</anchor>
      <arglist>(const WeakNodeList &amp;weak_nodes) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_waitable_handle</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>ad7cd8aaf5ffc2f7ac495df75d20bd2d2</anchor>
      <arglist>(const rclcpp::Waitable::SharedPtr &amp;waitable) override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>add_handles_to_wait_set</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a51bafda801f8ddacb9985e9fda6056e0</anchor>
      <arglist>(rcl_wait_set_t *wait_set) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>get_next_subscription</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>ae7564d1ecf08f82866132e75a6162f20</anchor>
      <arglist>(rclcpp::AnyExecutable &amp;any_exec, const WeakNodeList &amp;weak_nodes) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>get_next_service</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a8c34e35c5daf73377062308f4e6adf7f</anchor>
      <arglist>(rclcpp::AnyExecutable &amp;any_exec, const WeakNodeList &amp;weak_nodes) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>get_next_client</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>ace53477fa00ad1d1e232cda33db38d5b</anchor>
      <arglist>(rclcpp::AnyExecutable &amp;any_exec, const WeakNodeList &amp;weak_nodes) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>get_next_timer</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a7c0b95075f97dad5fda9b6f39af5090d</anchor>
      <arglist>(rclcpp::AnyExecutable &amp;any_exec, const WeakNodeList &amp;weak_nodes) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>get_next_waitable</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a22725eb99b1b729172cd2a62a24ff356</anchor>
      <arglist>(rclcpp::AnyExecutable &amp;any_exec, const WeakNodeList &amp;weak_nodes) override</arglist>
    </member>
    <member kind="function">
      <type>rcl_allocator_t</type>
      <name>get_allocator</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>aa7e40b96a4562e2039c07231271b4af4</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>number_of_ready_subscriptions</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>ab070d01a562af57d2f5c133222a3efdb</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>number_of_ready_services</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a5065e2f8822268b61900d1c1cccdae1b</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>number_of_ready_events</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a5d5880bb5852a4f959b2f8fe9d9589ac</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>number_of_ready_clients</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a755e6c8c9a7dfb410b700f10fc4d0fa6</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>number_of_guard_conditions</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a984a9b8d8a10d34f21195365c84fa7b0</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>number_of_ready_timers</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a31a7d09fb2dff7a32d0e122da7774465</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>number_of_waitables</name>
      <anchorfile>classrclcpp_1_1memory__strategies_1_1allocator__memory__strategy_1_1AllocatorMemoryStrategy.html</anchorfile>
      <anchor>a3d4e69410dd03382da9eaa7625fe7d21</anchor>
      <arglist>() const override</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::AnyExecutable</name>
    <filename>structrclcpp_1_1AnyExecutable.html</filename>
    <member kind="function">
      <type></type>
      <name>AnyExecutable</name>
      <anchorfile>structrclcpp_1_1AnyExecutable.html</anchorfile>
      <anchor>af51514a8682f8cfedf4f5b92885fb5a7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~AnyExecutable</name>
      <anchorfile>structrclcpp_1_1AnyExecutable.html</anchorfile>
      <anchor>a57bf2fd5a3f3e029c74823561b2380a5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::SubscriptionBase::SharedPtr</type>
      <name>subscription</name>
      <anchorfile>structrclcpp_1_1AnyExecutable.html</anchorfile>
      <anchor>a6a6d9ee2732f5e67f3024dcc544bd795</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::TimerBase::SharedPtr</type>
      <name>timer</name>
      <anchorfile>structrclcpp_1_1AnyExecutable.html</anchorfile>
      <anchor>a8045bf50e73db9e881320dfec99a5ba2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::ServiceBase::SharedPtr</type>
      <name>service</name>
      <anchorfile>structrclcpp_1_1AnyExecutable.html</anchorfile>
      <anchor>ae657581d4e8c4f6793e2fe6fab704b7a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::ClientBase::SharedPtr</type>
      <name>client</name>
      <anchorfile>structrclcpp_1_1AnyExecutable.html</anchorfile>
      <anchor>a0d111eef3ddfdeba4840fd851287ecf6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::Waitable::SharedPtr</type>
      <name>waitable</name>
      <anchorfile>structrclcpp_1_1AnyExecutable.html</anchorfile>
      <anchor>aee383407c1359544a12de05da2cf1b43</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::CallbackGroup::SharedPtr</type>
      <name>callback_group</name>
      <anchorfile>structrclcpp_1_1AnyExecutable.html</anchorfile>
      <anchor>aaf02d841472447671b041f54fbf4997d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::node_interfaces::NodeBaseInterface::SharedPtr</type>
      <name>node_base</name>
      <anchorfile>structrclcpp_1_1AnyExecutable.html</anchorfile>
      <anchor>ab06637ec7062a2bdb838164448f89332</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::AnyServiceCallback</name>
    <filename>classrclcpp_1_1AnyServiceCallback.html</filename>
    <templarg></templarg>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ab45b6365b3a5b90da42664bdd1ce977c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a70690b36adee59be69dd9b38bbc32788</anchor>
      <arglist>(const AnyServiceCallback &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>aff8f6483ee627db72b1b719f5d031611</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; typename ServiceT::Request &gt; request, std::shared_ptr&lt; typename ServiceT::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>register_callback_for_tracing</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a59debb390def627b6d8830105833f5a7</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AnyServiceCallback&lt; rcl_interfaces::srv::DescribeParameters &gt;</name>
    <filename>classrclcpp_1_1AnyServiceCallback.html</filename>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ab45b6365b3a5b90da42664bdd1ce977c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a70690b36adee59be69dd9b38bbc32788</anchor>
      <arglist>(const AnyServiceCallback &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>aff8f6483ee627db72b1b719f5d031611</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; typename rcl_interfaces::srv::DescribeParameters ::Request &gt; request, std::shared_ptr&lt; typename rcl_interfaces::srv::DescribeParameters ::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>register_callback_for_tracing</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a59debb390def627b6d8830105833f5a7</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AnyServiceCallback&lt; rcl_interfaces::srv::GetParameters &gt;</name>
    <filename>classrclcpp_1_1AnyServiceCallback.html</filename>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ab45b6365b3a5b90da42664bdd1ce977c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a70690b36adee59be69dd9b38bbc32788</anchor>
      <arglist>(const AnyServiceCallback &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>aff8f6483ee627db72b1b719f5d031611</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameters ::Request &gt; request, std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameters ::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>register_callback_for_tracing</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a59debb390def627b6d8830105833f5a7</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AnyServiceCallback&lt; rcl_interfaces::srv::GetParameterTypes &gt;</name>
    <filename>classrclcpp_1_1AnyServiceCallback.html</filename>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ab45b6365b3a5b90da42664bdd1ce977c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a70690b36adee59be69dd9b38bbc32788</anchor>
      <arglist>(const AnyServiceCallback &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>aff8f6483ee627db72b1b719f5d031611</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameterTypes ::Request &gt; request, std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameterTypes ::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>register_callback_for_tracing</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a59debb390def627b6d8830105833f5a7</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AnyServiceCallback&lt; rcl_interfaces::srv::ListParameters &gt;</name>
    <filename>classrclcpp_1_1AnyServiceCallback.html</filename>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ab45b6365b3a5b90da42664bdd1ce977c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a70690b36adee59be69dd9b38bbc32788</anchor>
      <arglist>(const AnyServiceCallback &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>aff8f6483ee627db72b1b719f5d031611</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; typename rcl_interfaces::srv::ListParameters ::Request &gt; request, std::shared_ptr&lt; typename rcl_interfaces::srv::ListParameters ::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>register_callback_for_tracing</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a59debb390def627b6d8830105833f5a7</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AnyServiceCallback&lt; rcl_interfaces::srv::SetParameters &gt;</name>
    <filename>classrclcpp_1_1AnyServiceCallback.html</filename>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ab45b6365b3a5b90da42664bdd1ce977c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a70690b36adee59be69dd9b38bbc32788</anchor>
      <arglist>(const AnyServiceCallback &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>aff8f6483ee627db72b1b719f5d031611</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; typename rcl_interfaces::srv::SetParameters ::Request &gt; request, std::shared_ptr&lt; typename rcl_interfaces::srv::SetParameters ::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>register_callback_for_tracing</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a59debb390def627b6d8830105833f5a7</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AnyServiceCallback&lt; rcl_interfaces::srv::SetParametersAtomically &gt;</name>
    <filename>classrclcpp_1_1AnyServiceCallback.html</filename>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ab45b6365b3a5b90da42664bdd1ce977c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AnyServiceCallback</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a70690b36adee59be69dd9b38bbc32788</anchor>
      <arglist>(const AnyServiceCallback &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>ad327a96aee572b3c2371204f041bd1c5</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>aff8f6483ee627db72b1b719f5d031611</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; typename rcl_interfaces::srv::SetParametersAtomically ::Request &gt; request, std::shared_ptr&lt; typename rcl_interfaces::srv::SetParametersAtomically ::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>register_callback_for_tracing</name>
      <anchorfile>classrclcpp_1_1AnyServiceCallback.html</anchorfile>
      <anchor>a59debb390def627b6d8830105833f5a7</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::AnySubscriptionCallback</name>
    <filename>classrclcpp_1_1AnySubscriptionCallback.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <member kind="function">
      <type></type>
      <name>AnySubscriptionCallback</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a1fafbece8be547ed88e8bf7f1fe85aa5</anchor>
      <arglist>(std::shared_ptr&lt; Alloc &gt; allocator)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AnySubscriptionCallback</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>abfa114dcb30864a35cb96cabddb9b2b7</anchor>
      <arglist>(const AnySubscriptionCallback &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>aa4a63471ed8f6234582adaf944c88a35</anchor>
      <arglist>(std::shared_ptr&lt; MessageT &gt; message, const rclcpp::MessageInfo &amp;message_info)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch_intra_process</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a0f9360b94e608313fa1147e7bb476466</anchor>
      <arglist>(ConstMessageSharedPtr message, const rclcpp::MessageInfo &amp;message_info)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch_intra_process</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a67f008667a9bec4282654154c5a051b0</anchor>
      <arglist>(MessageUniquePtr message, const rclcpp::MessageInfo &amp;message_info)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>use_take_shared_method</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>aaef376cd7ab1b19ef160fffe40c5cca6</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>register_callback_for_tracing</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a3289e1e88cd27c02935a36db0ce3bb82</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AnySubscriptionCallback&lt; CallbackMessageT, std::allocator&lt; void &gt; &gt;</name>
    <filename>classrclcpp_1_1AnySubscriptionCallback.html</filename>
    <member kind="function">
      <type></type>
      <name>AnySubscriptionCallback</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a1fafbece8be547ed88e8bf7f1fe85aa5</anchor>
      <arglist>(std::shared_ptr&lt; std::allocator&lt; void &gt; &gt; allocator)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AnySubscriptionCallback</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>abfa114dcb30864a35cb96cabddb9b2b7</anchor>
      <arglist>(const AnySubscriptionCallback &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>aa4a63471ed8f6234582adaf944c88a35</anchor>
      <arglist>(std::shared_ptr&lt; CallbackMessageT &gt; message, const rclcpp::MessageInfo &amp;message_info)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch_intra_process</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a0f9360b94e608313fa1147e7bb476466</anchor>
      <arglist>(ConstMessageSharedPtr message, const rclcpp::MessageInfo &amp;message_info)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch_intra_process</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a67f008667a9bec4282654154c5a051b0</anchor>
      <arglist>(MessageUniquePtr message, const rclcpp::MessageInfo &amp;message_info)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>use_take_shared_method</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>aaef376cd7ab1b19ef160fffe40c5cca6</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>register_callback_for_tracing</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a3289e1e88cd27c02935a36db0ce3bb82</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AnySubscriptionCallback&lt; MessageT, std::allocator&lt; void &gt; &gt;</name>
    <filename>classrclcpp_1_1AnySubscriptionCallback.html</filename>
    <member kind="function">
      <type></type>
      <name>AnySubscriptionCallback</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a1fafbece8be547ed88e8bf7f1fe85aa5</anchor>
      <arglist>(std::shared_ptr&lt; std::allocator&lt; void &gt; &gt; allocator)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AnySubscriptionCallback</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>abfa114dcb30864a35cb96cabddb9b2b7</anchor>
      <arglist>(const AnySubscriptionCallback &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a159041aa620f794f440bb4a84f8704f0</anchor>
      <arglist>(CallbackT callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>aa4a63471ed8f6234582adaf944c88a35</anchor>
      <arglist>(std::shared_ptr&lt; MessageT &gt; message, const rclcpp::MessageInfo &amp;message_info)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch_intra_process</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a0f9360b94e608313fa1147e7bb476466</anchor>
      <arglist>(ConstMessageSharedPtr message, const rclcpp::MessageInfo &amp;message_info)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>dispatch_intra_process</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a67f008667a9bec4282654154c5a051b0</anchor>
      <arglist>(MessageUniquePtr message, const rclcpp::MessageInfo &amp;message_info)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>use_take_shared_method</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>aaef376cd7ab1b19ef160fffe40c5cca6</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>register_callback_for_tracing</name>
      <anchorfile>classrclcpp_1_1AnySubscriptionCallback.html</anchorfile>
      <anchor>a3289e1e88cd27c02935a36db0ce3bb82</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::function_traits::arity_comparator</name>
    <filename>structrclcpp_1_1function__traits_1_1arity__comparator.html</filename>
    <templarg>Arity</templarg>
    <templarg></templarg>
    <base>integral_constant&lt; bool,(Arity==function_traits&lt; FunctorT &gt;::arity)&gt;</base>
  </compound>
  <compound kind="class">
    <name>rclcpp::AsyncParametersClient</name>
    <filename>classrclcpp_1_1AsyncParametersClient.html</filename>
    <member kind="function">
      <type></type>
      <name>AsyncParametersClient</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>abb602f2624c59ac3198ebf948bde332f</anchor>
      <arglist>(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface, const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface, const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface, const std::string &amp;remote_node_name=&quot;&quot;, const rmw_qos_profile_t &amp;qos_profile=rmw_qos_profile_parameters, rclcpp::CallbackGroup::SharedPtr group=nullptr)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AsyncParametersClient</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>ae99da8a322035788b209f19884c7341c</anchor>
      <arglist>(const rclcpp::Node::SharedPtr node, const std::string &amp;remote_node_name=&quot;&quot;, const rmw_qos_profile_t &amp;qos_profile=rmw_qos_profile_parameters, rclcpp::CallbackGroup::SharedPtr group=nullptr)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AsyncParametersClient</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>a93cda4036951d27deeb257ffe01971ba</anchor>
      <arglist>(rclcpp::Node *node, const std::string &amp;remote_node_name=&quot;&quot;, const rmw_qos_profile_t &amp;qos_profile=rmw_qos_profile_parameters, rclcpp::CallbackGroup::SharedPtr group=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_future&lt; std::vector&lt; rclcpp::Parameter &gt; &gt;</type>
      <name>get_parameters</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>a411ce115fbb76cba53eabae0b89f8337</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;names, std::function&lt; void(std::shared_future&lt; std::vector&lt; rclcpp::Parameter &gt;&gt;) &gt; callback=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_future&lt; std::vector&lt; rclcpp::ParameterType &gt; &gt;</type>
      <name>get_parameter_types</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>af852956ffade2720535880b8e2510317</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;names, std::function&lt; void(std::shared_future&lt; std::vector&lt; rclcpp::ParameterType &gt;&gt;) &gt; callback=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_future&lt; std::vector&lt; rcl_interfaces::msg::SetParametersResult &gt; &gt;</type>
      <name>set_parameters</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>a38b2f54f4b0535f19e6f92c6b651af10</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameters, std::function&lt; void(std::shared_future&lt; std::vector&lt; rcl_interfaces::msg::SetParametersResult &gt;&gt;) &gt; callback=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_future&lt; rcl_interfaces::msg::SetParametersResult &gt;</type>
      <name>set_parameters_atomically</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>a192ef60deac868dd9ad1071fc6e88c4e</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameters, std::function&lt; void(std::shared_future&lt; rcl_interfaces::msg::SetParametersResult &gt;) &gt; callback=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_future&lt; rcl_interfaces::msg::ListParametersResult &gt;</type>
      <name>list_parameters</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>ab4843d0253684bfe5cf08d71b3bcc285</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;prefixes, uint64_t depth, std::function&lt; void(std::shared_future&lt; rcl_interfaces::msg::ListParametersResult &gt;) &gt; callback=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Subscription&lt; rcl_interfaces::msg::ParameterEvent &gt;::SharedPtr</type>
      <name>on_parameter_event</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>a18b7407d5d324b9680887bcb77578130</anchor>
      <arglist>(CallbackT &amp;&amp;callback, const rclcpp::QoS &amp;qos=(rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events))), const rclcpp::SubscriptionOptionsWithAllocator&lt; AllocatorT &gt; &amp;options=(rclcpp::SubscriptionOptionsWithAllocator&lt; AllocatorT &gt;()))</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>service_is_ready</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>acf603c02489da23935102f89590303df</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>wait_for_service</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>a36ce0351e24b4e2eb1a6d7c2a9e7215b</anchor>
      <arglist>(std::chrono::duration&lt; RepT, RatioT &gt; timeout=std::chrono::duration&lt; RepT, RatioT &gt;(-1))</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static rclcpp::Subscription&lt; rcl_interfaces::msg::ParameterEvent &gt;::SharedPtr</type>
      <name>on_parameter_event</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>a6ae24be6e6b4a37fd8aa323f3da24869</anchor>
      <arglist>(NodeT &amp;&amp;node, CallbackT &amp;&amp;callback, const rclcpp::QoS &amp;qos=(rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events))), const rclcpp::SubscriptionOptionsWithAllocator&lt; AllocatorT &gt; &amp;options=(rclcpp::SubscriptionOptionsWithAllocator&lt; AllocatorT &gt;()))</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>bool</type>
      <name>wait_for_service_nanoseconds</name>
      <anchorfile>classrclcpp_1_1AsyncParametersClient.html</anchorfile>
      <anchor>a9a3d13e6aa20f36b65c91dffa2fa3382</anchor>
      <arglist>(std::chrono::nanoseconds timeout)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::experimental::buffers::BufferImplementationBase</name>
    <filename>classrclcpp_1_1experimental_1_1buffers_1_1BufferImplementationBase.html</filename>
    <templarg></templarg>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~BufferImplementationBase</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1BufferImplementationBase.html</anchorfile>
      <anchor>a3004b58e95c1fc08f3e39fb4bb3b2ff4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual BufferT</type>
      <name>dequeue</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1BufferImplementationBase.html</anchorfile>
      <anchor>ae53d0358e1c6cfca4aeb011ddc2a8665</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>enqueue</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1BufferImplementationBase.html</anchorfile>
      <anchor>a29ac6c750d8eb513d85c9632d885c2d8</anchor>
      <arglist>(BufferT request)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>clear</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1BufferImplementationBase.html</anchorfile>
      <anchor>a27219663164ca769f8be576818a772cb</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>has_data</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1BufferImplementationBase.html</anchorfile>
      <anchor>a38b551704e3ea22962a1ee050eff0cc8</anchor>
      <arglist>() const =0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::CallbackGroup</name>
    <filename>classrclcpp_1_1CallbackGroup.html</filename>
    <member kind="function">
      <type></type>
      <name>CallbackGroup</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a77e223db78c3ef9b0aff1d9eb81b2d6e</anchor>
      <arglist>(CallbackGroupType group_type)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::SubscriptionBase::SharedPtr</type>
      <name>find_subscription_ptrs_if</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a4a548612a5bab6512a8223ae18b1fc50</anchor>
      <arglist>(Function func) const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::TimerBase::SharedPtr</type>
      <name>find_timer_ptrs_if</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>aaf087dde6c9b22166ff444f4fd463f9b</anchor>
      <arglist>(Function func) const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::ServiceBase::SharedPtr</type>
      <name>find_service_ptrs_if</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a7cff83a9303ebcca737920e869dd1570</anchor>
      <arglist>(Function func) const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::ClientBase::SharedPtr</type>
      <name>find_client_ptrs_if</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a961fd2e9d4c4cdb2e1204d856b34239c</anchor>
      <arglist>(Function func) const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Waitable::SharedPtr</type>
      <name>find_waitable_ptrs_if</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>aa9546c1538e13fba2c71f896af726de4</anchor>
      <arglist>(Function func) const</arglist>
    </member>
    <member kind="function">
      <type>std::atomic_bool &amp;</type>
      <name>can_be_taken_from</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a0d860dc93b9aefeb86a3601c57c0db49</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const CallbackGroupType &amp;</type>
      <name>type</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>ac404f5edddd418fffd552dace1759c3b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>add_publisher</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>af4e2ec03c86237a1940bace511bcb64e</anchor>
      <arglist>(const rclcpp::PublisherBase::SharedPtr publisher_ptr)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>add_subscription</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a5f92b3c005d05f000914147d482b697c</anchor>
      <arglist>(const rclcpp::SubscriptionBase::SharedPtr subscription_ptr)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>add_timer</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a80b7f2f67d52ebbb84b28a80ecdb778b</anchor>
      <arglist>(const rclcpp::TimerBase::SharedPtr timer_ptr)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>add_service</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a9a336a680733adc20e9ab8f794f14d99</anchor>
      <arglist>(const rclcpp::ServiceBase::SharedPtr service_ptr)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>add_client</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>aed689c33b8ae51d5a9da41f15898e089</anchor>
      <arglist>(const rclcpp::ClientBase::SharedPtr client_ptr)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>add_waitable</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a16d3d87d2c0daefb51f6c99cdcb4139d</anchor>
      <arglist>(const rclcpp::Waitable::SharedPtr waitable_ptr)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>remove_waitable</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>ae43641e4512087a6b6d231d77de12ff5</anchor>
      <arglist>(const rclcpp::Waitable::SharedPtr waitable_ptr) noexcept</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>CallbackGroupType</type>
      <name>type_</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a92c9f9cb9ae95d1579ea8f2226b16a5a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::mutex</type>
      <name>mutex_</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a6ef4bdd15ce25b67611748f48d69a4f9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; rclcpp::SubscriptionBase::WeakPtr &gt;</type>
      <name>subscription_ptrs_</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a35860e89654fb5b671ce7560fcbc090d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; rclcpp::TimerBase::WeakPtr &gt;</type>
      <name>timer_ptrs_</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a9c4b5ebb50868b9db127067ad6edabab</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; rclcpp::ServiceBase::WeakPtr &gt;</type>
      <name>service_ptrs_</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>afb2a12b02801bb342d7b83d33f2f33f2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; rclcpp::ClientBase::WeakPtr &gt;</type>
      <name>client_ptrs_</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a1900a8f2a84b67d856783c0122f6aac0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; rclcpp::Waitable::WeakPtr &gt;</type>
      <name>waitable_ptrs_</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a12d701430ac327f3772d7cbabf7ab18c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::atomic_bool</type>
      <name>can_be_taken_from_</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a12b4271ba9e0ed7de1f64f82b07bc2b9</anchor>
      <arglist></arglist>
    </member>
    <member kind="friend">
      <type>friend class</type>
      <name>rclcpp::node_interfaces::NodeServices</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>a63f67ef2e5e2d8f593a64b883c8ea3df</anchor>
      <arglist></arglist>
    </member>
    <member kind="friend">
      <type>friend class</type>
      <name>rclcpp::node_interfaces::NodeTimers</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>aeeb1ac7e4e912bb8dc7aa1813535b0ed</anchor>
      <arglist></arglist>
    </member>
    <member kind="friend">
      <type>friend class</type>
      <name>rclcpp::node_interfaces::NodeTopics</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>ad0616a6ca4b2217d0dc1e94877547a77</anchor>
      <arglist></arglist>
    </member>
    <member kind="friend">
      <type>friend class</type>
      <name>rclcpp::node_interfaces::NodeWaitables</name>
      <anchorfile>classrclcpp_1_1CallbackGroup.html</anchorfile>
      <anchor>ab6d6e6e119688fe097e360f8cd3c6759</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::function_traits::check_arguments</name>
    <filename>structrclcpp_1_1function__traits_1_1check__arguments.html</filename>
    <templarg></templarg>
    <templarg>Args</templarg>
    <base>is_same&lt; function_traits&lt; FunctorT &gt;::arguments, std::tuple&lt; Args ... &gt; &gt;</base>
  </compound>
  <compound kind="class">
    <name>rclcpp::Client</name>
    <filename>classrclcpp_1_1Client.html</filename>
    <templarg></templarg>
    <base>rclcpp::ClientBase</base>
    <member kind="typedef">
      <type>typename ServiceT::Request::SharedPtr</type>
      <name>SharedRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>acfa72f93a845c3b7deb385f91c4e55be</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename ServiceT::Response::SharedPtr</type>
      <name>SharedResponse</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a2d8ad27d92e21548e80a106456e9c701</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; SharedResponse &gt;</type>
      <name>Promise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad7ab1c7bc6f3c5f35ca883706a41f000</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>PromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aab2c0b11463f8168b0f3400d330a05b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; Promise &gt;</type>
      <name>SharedPromise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad3dfea938e183e9607a3ec0baa822a2f</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; PromiseWithRequest &gt;</type>
      <name>SharedPromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aeddb0ec9cdd39c6b0a3508431691a28c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; SharedResponse &gt;</type>
      <name>SharedFuture</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac7203d297201a83411a4274ccf9a1e2a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>SharedFutureWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abca86c6b203cd9eab70554d8491d2a67</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFuture)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aa40dc013051306b91b558f4d701214d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFutureWithRequest)&gt;</type>
      <name>CallbackWithRequestType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac45b3da8a45ab490a95418d3c1cff16d</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ae309a79c40cc329cd6d3360271d1b997</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph, const std::string &amp;service_name, rcl_client_options_t &amp;client_options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abb39c469124b9012fbc07ec8fc113237</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a01895d118ee1ab910b00de5456a333fc</anchor>
      <arglist>(typename ServiceT::Response &amp;response_out, rmw_request_id_t &amp;request_header_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a1f9ef5ef0628e7b00d44733624cdc05b</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a0fb948d3ca948d6a5e453ec01f9e5764</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a812ead8610b4472f492cc500718fdeab</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; response) override</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a7567297f43b72f96e8ec57fa7ff2f4e1</anchor>
      <arglist>(SharedRequest request)</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a62e48edd618bcb73538bfdc3ee3d5e63</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
    <member kind="function">
      <type>SharedFutureWithRequest</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a95d4fe0e53735f5fe751d92fd8ea3442</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Client&lt; rcl_interfaces::srv::DescribeParameters &gt;</name>
    <filename>classrclcpp_1_1Client.html</filename>
    <base>rclcpp::ClientBase</base>
    <member kind="typedef">
      <type>typename rcl_interfaces::srv::DescribeParameters ::Request::SharedPtr</type>
      <name>SharedRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>acfa72f93a845c3b7deb385f91c4e55be</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename rcl_interfaces::srv::DescribeParameters ::Response::SharedPtr</type>
      <name>SharedResponse</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a2d8ad27d92e21548e80a106456e9c701</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; SharedResponse &gt;</type>
      <name>Promise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad7ab1c7bc6f3c5f35ca883706a41f000</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>PromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aab2c0b11463f8168b0f3400d330a05b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; Promise &gt;</type>
      <name>SharedPromise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad3dfea938e183e9607a3ec0baa822a2f</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; PromiseWithRequest &gt;</type>
      <name>SharedPromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aeddb0ec9cdd39c6b0a3508431691a28c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; SharedResponse &gt;</type>
      <name>SharedFuture</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac7203d297201a83411a4274ccf9a1e2a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>SharedFutureWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abca86c6b203cd9eab70554d8491d2a67</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFuture)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aa40dc013051306b91b558f4d701214d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFutureWithRequest)&gt;</type>
      <name>CallbackWithRequestType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac45b3da8a45ab490a95418d3c1cff16d</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ae309a79c40cc329cd6d3360271d1b997</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph, const std::string &amp;service_name, rcl_client_options_t &amp;client_options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abb39c469124b9012fbc07ec8fc113237</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a01895d118ee1ab910b00de5456a333fc</anchor>
      <arglist>(typename rcl_interfaces::srv::DescribeParameters ::Response &amp;response_out, rmw_request_id_t &amp;request_header_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a1f9ef5ef0628e7b00d44733624cdc05b</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a0fb948d3ca948d6a5e453ec01f9e5764</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a812ead8610b4472f492cc500718fdeab</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; response) override</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a7567297f43b72f96e8ec57fa7ff2f4e1</anchor>
      <arglist>(SharedRequest request)</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a62e48edd618bcb73538bfdc3ee3d5e63</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
    <member kind="function">
      <type>SharedFutureWithRequest</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a95d4fe0e53735f5fe751d92fd8ea3442</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Client&lt; rcl_interfaces::srv::GetParameters &gt;</name>
    <filename>classrclcpp_1_1Client.html</filename>
    <base>rclcpp::ClientBase</base>
    <member kind="typedef">
      <type>typename rcl_interfaces::srv::GetParameters ::Request::SharedPtr</type>
      <name>SharedRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>acfa72f93a845c3b7deb385f91c4e55be</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename rcl_interfaces::srv::GetParameters ::Response::SharedPtr</type>
      <name>SharedResponse</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a2d8ad27d92e21548e80a106456e9c701</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; SharedResponse &gt;</type>
      <name>Promise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad7ab1c7bc6f3c5f35ca883706a41f000</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>PromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aab2c0b11463f8168b0f3400d330a05b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; Promise &gt;</type>
      <name>SharedPromise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad3dfea938e183e9607a3ec0baa822a2f</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; PromiseWithRequest &gt;</type>
      <name>SharedPromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aeddb0ec9cdd39c6b0a3508431691a28c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; SharedResponse &gt;</type>
      <name>SharedFuture</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac7203d297201a83411a4274ccf9a1e2a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>SharedFutureWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abca86c6b203cd9eab70554d8491d2a67</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFuture)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aa40dc013051306b91b558f4d701214d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFutureWithRequest)&gt;</type>
      <name>CallbackWithRequestType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac45b3da8a45ab490a95418d3c1cff16d</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ae309a79c40cc329cd6d3360271d1b997</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph, const std::string &amp;service_name, rcl_client_options_t &amp;client_options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abb39c469124b9012fbc07ec8fc113237</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a01895d118ee1ab910b00de5456a333fc</anchor>
      <arglist>(typename rcl_interfaces::srv::GetParameters ::Response &amp;response_out, rmw_request_id_t &amp;request_header_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a1f9ef5ef0628e7b00d44733624cdc05b</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a0fb948d3ca948d6a5e453ec01f9e5764</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a812ead8610b4472f492cc500718fdeab</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; response) override</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a7567297f43b72f96e8ec57fa7ff2f4e1</anchor>
      <arglist>(SharedRequest request)</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a62e48edd618bcb73538bfdc3ee3d5e63</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
    <member kind="function">
      <type>SharedFutureWithRequest</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a95d4fe0e53735f5fe751d92fd8ea3442</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Client&lt; rcl_interfaces::srv::GetParameterTypes &gt;</name>
    <filename>classrclcpp_1_1Client.html</filename>
    <base>rclcpp::ClientBase</base>
    <member kind="typedef">
      <type>typename rcl_interfaces::srv::GetParameterTypes ::Request::SharedPtr</type>
      <name>SharedRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>acfa72f93a845c3b7deb385f91c4e55be</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename rcl_interfaces::srv::GetParameterTypes ::Response::SharedPtr</type>
      <name>SharedResponse</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a2d8ad27d92e21548e80a106456e9c701</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; SharedResponse &gt;</type>
      <name>Promise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad7ab1c7bc6f3c5f35ca883706a41f000</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>PromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aab2c0b11463f8168b0f3400d330a05b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; Promise &gt;</type>
      <name>SharedPromise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad3dfea938e183e9607a3ec0baa822a2f</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; PromiseWithRequest &gt;</type>
      <name>SharedPromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aeddb0ec9cdd39c6b0a3508431691a28c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; SharedResponse &gt;</type>
      <name>SharedFuture</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac7203d297201a83411a4274ccf9a1e2a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>SharedFutureWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abca86c6b203cd9eab70554d8491d2a67</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFuture)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aa40dc013051306b91b558f4d701214d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFutureWithRequest)&gt;</type>
      <name>CallbackWithRequestType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac45b3da8a45ab490a95418d3c1cff16d</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ae309a79c40cc329cd6d3360271d1b997</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph, const std::string &amp;service_name, rcl_client_options_t &amp;client_options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abb39c469124b9012fbc07ec8fc113237</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a01895d118ee1ab910b00de5456a333fc</anchor>
      <arglist>(typename rcl_interfaces::srv::GetParameterTypes ::Response &amp;response_out, rmw_request_id_t &amp;request_header_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a1f9ef5ef0628e7b00d44733624cdc05b</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a0fb948d3ca948d6a5e453ec01f9e5764</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a812ead8610b4472f492cc500718fdeab</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; response) override</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a7567297f43b72f96e8ec57fa7ff2f4e1</anchor>
      <arglist>(SharedRequest request)</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a62e48edd618bcb73538bfdc3ee3d5e63</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
    <member kind="function">
      <type>SharedFutureWithRequest</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a95d4fe0e53735f5fe751d92fd8ea3442</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Client&lt; rcl_interfaces::srv::ListParameters &gt;</name>
    <filename>classrclcpp_1_1Client.html</filename>
    <base>rclcpp::ClientBase</base>
    <member kind="typedef">
      <type>typename rcl_interfaces::srv::ListParameters ::Request::SharedPtr</type>
      <name>SharedRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>acfa72f93a845c3b7deb385f91c4e55be</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename rcl_interfaces::srv::ListParameters ::Response::SharedPtr</type>
      <name>SharedResponse</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a2d8ad27d92e21548e80a106456e9c701</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; SharedResponse &gt;</type>
      <name>Promise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad7ab1c7bc6f3c5f35ca883706a41f000</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>PromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aab2c0b11463f8168b0f3400d330a05b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; Promise &gt;</type>
      <name>SharedPromise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad3dfea938e183e9607a3ec0baa822a2f</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; PromiseWithRequest &gt;</type>
      <name>SharedPromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aeddb0ec9cdd39c6b0a3508431691a28c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; SharedResponse &gt;</type>
      <name>SharedFuture</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac7203d297201a83411a4274ccf9a1e2a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>SharedFutureWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abca86c6b203cd9eab70554d8491d2a67</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFuture)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aa40dc013051306b91b558f4d701214d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFutureWithRequest)&gt;</type>
      <name>CallbackWithRequestType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac45b3da8a45ab490a95418d3c1cff16d</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ae309a79c40cc329cd6d3360271d1b997</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph, const std::string &amp;service_name, rcl_client_options_t &amp;client_options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abb39c469124b9012fbc07ec8fc113237</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a01895d118ee1ab910b00de5456a333fc</anchor>
      <arglist>(typename rcl_interfaces::srv::ListParameters ::Response &amp;response_out, rmw_request_id_t &amp;request_header_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a1f9ef5ef0628e7b00d44733624cdc05b</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a0fb948d3ca948d6a5e453ec01f9e5764</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a812ead8610b4472f492cc500718fdeab</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; response) override</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a7567297f43b72f96e8ec57fa7ff2f4e1</anchor>
      <arglist>(SharedRequest request)</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a62e48edd618bcb73538bfdc3ee3d5e63</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
    <member kind="function">
      <type>SharedFutureWithRequest</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a95d4fe0e53735f5fe751d92fd8ea3442</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Client&lt; rcl_interfaces::srv::SetParameters &gt;</name>
    <filename>classrclcpp_1_1Client.html</filename>
    <base>rclcpp::ClientBase</base>
    <member kind="typedef">
      <type>typename rcl_interfaces::srv::SetParameters ::Request::SharedPtr</type>
      <name>SharedRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>acfa72f93a845c3b7deb385f91c4e55be</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename rcl_interfaces::srv::SetParameters ::Response::SharedPtr</type>
      <name>SharedResponse</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a2d8ad27d92e21548e80a106456e9c701</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; SharedResponse &gt;</type>
      <name>Promise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad7ab1c7bc6f3c5f35ca883706a41f000</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>PromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aab2c0b11463f8168b0f3400d330a05b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; Promise &gt;</type>
      <name>SharedPromise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad3dfea938e183e9607a3ec0baa822a2f</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; PromiseWithRequest &gt;</type>
      <name>SharedPromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aeddb0ec9cdd39c6b0a3508431691a28c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; SharedResponse &gt;</type>
      <name>SharedFuture</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac7203d297201a83411a4274ccf9a1e2a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>SharedFutureWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abca86c6b203cd9eab70554d8491d2a67</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFuture)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aa40dc013051306b91b558f4d701214d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFutureWithRequest)&gt;</type>
      <name>CallbackWithRequestType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac45b3da8a45ab490a95418d3c1cff16d</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ae309a79c40cc329cd6d3360271d1b997</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph, const std::string &amp;service_name, rcl_client_options_t &amp;client_options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abb39c469124b9012fbc07ec8fc113237</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a01895d118ee1ab910b00de5456a333fc</anchor>
      <arglist>(typename rcl_interfaces::srv::SetParameters ::Response &amp;response_out, rmw_request_id_t &amp;request_header_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a1f9ef5ef0628e7b00d44733624cdc05b</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a0fb948d3ca948d6a5e453ec01f9e5764</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a812ead8610b4472f492cc500718fdeab</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; response) override</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a7567297f43b72f96e8ec57fa7ff2f4e1</anchor>
      <arglist>(SharedRequest request)</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a62e48edd618bcb73538bfdc3ee3d5e63</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
    <member kind="function">
      <type>SharedFutureWithRequest</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a95d4fe0e53735f5fe751d92fd8ea3442</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Client&lt; rcl_interfaces::srv::SetParametersAtomically &gt;</name>
    <filename>classrclcpp_1_1Client.html</filename>
    <base>rclcpp::ClientBase</base>
    <member kind="typedef">
      <type>typename rcl_interfaces::srv::SetParametersAtomically ::Request::SharedPtr</type>
      <name>SharedRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>acfa72f93a845c3b7deb385f91c4e55be</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename rcl_interfaces::srv::SetParametersAtomically ::Response::SharedPtr</type>
      <name>SharedResponse</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a2d8ad27d92e21548e80a106456e9c701</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; SharedResponse &gt;</type>
      <name>Promise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad7ab1c7bc6f3c5f35ca883706a41f000</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::promise&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>PromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aab2c0b11463f8168b0f3400d330a05b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; Promise &gt;</type>
      <name>SharedPromise</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ad3dfea938e183e9607a3ec0baa822a2f</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; PromiseWithRequest &gt;</type>
      <name>SharedPromiseWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aeddb0ec9cdd39c6b0a3508431691a28c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; SharedResponse &gt;</type>
      <name>SharedFuture</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac7203d297201a83411a4274ccf9a1e2a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_future&lt; std::pair&lt; SharedRequest, SharedResponse &gt; &gt;</type>
      <name>SharedFutureWithRequest</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abca86c6b203cd9eab70554d8491d2a67</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFuture)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>aa40dc013051306b91b558f4d701214d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(SharedFutureWithRequest)&gt;</type>
      <name>CallbackWithRequestType</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ac45b3da8a45ab490a95418d3c1cff16d</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>ae309a79c40cc329cd6d3360271d1b997</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph, const std::string &amp;service_name, rcl_client_options_t &amp;client_options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Client</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>abb39c469124b9012fbc07ec8fc113237</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a01895d118ee1ab910b00de5456a333fc</anchor>
      <arglist>(typename rcl_interfaces::srv::SetParametersAtomically ::Response &amp;response_out, rmw_request_id_t &amp;request_header_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a1f9ef5ef0628e7b00d44733624cdc05b</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a0fb948d3ca948d6a5e453ec01f9e5764</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_response</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a812ead8610b4472f492cc500718fdeab</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; response) override</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a7567297f43b72f96e8ec57fa7ff2f4e1</anchor>
      <arglist>(SharedRequest request)</arglist>
    </member>
    <member kind="function">
      <type>SharedFuture</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a62e48edd618bcb73538bfdc3ee3d5e63</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
    <member kind="function">
      <type>SharedFutureWithRequest</type>
      <name>async_send_request</name>
      <anchorfile>classrclcpp_1_1Client.html</anchorfile>
      <anchor>a95d4fe0e53735f5fe751d92fd8ea3442</anchor>
      <arglist>(SharedRequest request, CallbackT &amp;&amp;cb)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::ClientBase</name>
    <filename>classrclcpp_1_1ClientBase.html</filename>
    <member kind="function">
      <type></type>
      <name>ClientBase</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>aa85dd4b61a7fb76b9e77c129ca4a324a</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ClientBase</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>a3b6eb6597b106ae0c6b735e9044ece3e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_type_erased_response</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>a22b06fe837366c08c7a4fa49a24e7fd7</anchor>
      <arglist>(void *response_out, rmw_request_id_t &amp;request_header_out)</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_service_name</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>ac232161c9e1a69bb6d96cb4a5d487719</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rcl_client_t &gt;</type>
      <name>get_client_handle</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>a61cea83f8896e18d925b6aca55009199</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; const rcl_client_t &gt;</type>
      <name>get_client_handle</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>a5ac1d438fdd09e9ba429ab3d9c5ee237</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>service_is_ready</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>a8b4b432338a460ceb26a7fa6ddd59e1d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>wait_for_service</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>aa8dc7698de300a0bbf08adb3f766d97f</anchor>
      <arglist>(std::chrono::duration&lt; RepT, RatioT &gt; timeout=std::chrono::duration&lt; RepT, RatioT &gt;(-1))</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::shared_ptr&lt; void &gt;</type>
      <name>create_response</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>ad83f47f52ba7f11dd5029b48a920d1f3</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>afd4baacaba527a6dfca7f6611678e0b8</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>handle_response</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>a71e38d95c59a830ed2b9778f908afb35</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; response)=0</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>exchange_in_use_by_wait_set_state</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>a66f551fb43ebd4094409434419c48989</anchor>
      <arglist>(bool in_use_state)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>bool</type>
      <name>wait_for_service_nanoseconds</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>a491cc4685782d3a3f3128d87e6df4d0b</anchor>
      <arglist>(std::chrono::nanoseconds timeout)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>rcl_node_t *</type>
      <name>get_rcl_node_handle</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>a048966495a7b546bba340d69fd124c31</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>const rcl_node_t *</type>
      <name>get_rcl_node_handle</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>a4f8b96115d62816c223ef8b9b6d779e2</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::node_interfaces::NodeGraphInterface::WeakPtr</type>
      <name>node_graph_</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>afea9468a9303b45c82b35b792c432f9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rcl_node_t &gt;</type>
      <name>node_handle_</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>a87619c0fd257e3d5cd80219a508a9188</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Context &gt;</type>
      <name>context_</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>aff8a183c003161384b2e8629b5e50233</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rcl_client_t &gt;</type>
      <name>client_handle_</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>aa30bc127da91c8bbdc1968cac1832466</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::atomic&lt; bool &gt;</type>
      <name>in_use_by_wait_set_</name>
      <anchorfile>classrclcpp_1_1ClientBase.html</anchorfile>
      <anchor>a528605b062a7018abdbc42d9497c480a</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Clock</name>
    <filename>classrclcpp_1_1Clock.html</filename>
    <member kind="function">
      <type></type>
      <name>Clock</name>
      <anchorfile>classrclcpp_1_1Clock.html</anchorfile>
      <anchor>abe5646eb46910ea5bda2486d082a31ab</anchor>
      <arglist>(rcl_clock_type_t clock_type=RCL_SYSTEM_TIME)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Clock</name>
      <anchorfile>classrclcpp_1_1Clock.html</anchorfile>
      <anchor>affff6cda7ae4ef55b7ea3159b512ff8f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Time</type>
      <name>now</name>
      <anchorfile>classrclcpp_1_1Clock.html</anchorfile>
      <anchor>a6375208429c40bd9d6e84b2b32557182</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>ros_time_is_active</name>
      <anchorfile>classrclcpp_1_1Clock.html</anchorfile>
      <anchor>a3bebc635619797c58c2da5dc9b6032a8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rcl_clock_t *</type>
      <name>get_clock_handle</name>
      <anchorfile>classrclcpp_1_1Clock.html</anchorfile>
      <anchor>ada613671b821b8c9aea77a1e98c995d7</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>rcl_clock_type_t</type>
      <name>get_clock_type</name>
      <anchorfile>classrclcpp_1_1Clock.html</anchorfile>
      <anchor>a75ab0eb8e83303fd9c8c49a9807440ea</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>std::mutex &amp;</type>
      <name>get_clock_mutex</name>
      <anchorfile>classrclcpp_1_1Clock.html</anchorfile>
      <anchor>a8988cc6a22e839236a89553af4347f31</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>JumpHandler::SharedPtr</type>
      <name>create_jump_callback</name>
      <anchorfile>classrclcpp_1_1Clock.html</anchorfile>
      <anchor>a706936e6e1263cbd4c509cce5dbb5fce</anchor>
      <arglist>(JumpHandler::pre_callback_t pre_callback, JumpHandler::post_callback_t post_callback, const rcl_jump_threshold_t &amp;threshold)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Context</name>
    <filename>classrclcpp_1_1Context.html</filename>
    <base>enable_shared_from_this&lt; Context &gt;</base>
    <member kind="typedef">
      <type>std::function&lt; void()&gt;</type>
      <name>OnShutdownCallback</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>a64bce7cbf8b92f3e0283077f26facf9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Context</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>a1b274e8c22d9f800a9a1527330f83f7f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Context</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>a087366f7387944185bf8c02f4f8873c7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>init</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>a6f938e3bd0a0f1eb3691d22c583ddfed</anchor>
      <arglist>(int argc, char const *const argv[], const rclcpp::InitOptions &amp;init_options=rclcpp::InitOptions())</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_valid</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>afab1efb0368a6d291c519cf90450762c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const rclcpp::InitOptions &amp;</type>
      <name>get_init_options</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>a7de1320462bbdfe0b7f07614be2d98e9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::InitOptions</type>
      <name>get_init_options</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>ab7df3a8380939883be7e8c4c240f6255</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>shutdown_reason</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>aea006131f2eeb6a0e6c567e305ba9755</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>shutdown</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>ac51bf15d106aa246001a4fdc15be7b2a</anchor>
      <arglist>(const std::string &amp;reason)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual OnShutdownCallback</type>
      <name>on_shutdown</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>a0d731d83e73a5b368c9f5bee1d2d4cb2</anchor>
      <arglist>(OnShutdownCallback callback)</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; OnShutdownCallback &gt; &amp;</type>
      <name>get_on_shutdown_callbacks</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>a43fd7326634fb77c060b3bb1b33fb6af</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; OnShutdownCallback &gt; &amp;</type>
      <name>get_on_shutdown_callbacks</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>acacf947e05b997ece54cd189e7cfe0c2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rcl_context_t &gt;</type>
      <name>get_rcl_context</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>a28596bd16aad20e057160f8a66d4697f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>sleep_for</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>adf25fc22a74daf5ce85f6bdde3b92aa2</anchor>
      <arglist>(const std::chrono::nanoseconds &amp;nanoseconds)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>interrupt_all_sleep_for</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>af659ad5489ffaf5e38f0db39c156d34a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rcl_guard_condition_t *</type>
      <name>get_interrupt_guard_condition</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>a79e1bee2976ea83ec93518b3bc359b78</anchor>
      <arglist>(rcl_wait_set_t *wait_set)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>release_interrupt_guard_condition</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>a48cfd20fea94c8e029d23af17e751e0a</anchor>
      <arglist>(rcl_wait_set_t *wait_set)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>release_interrupt_guard_condition</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>aa4c2162e98eca3b8790666896c78fd14</anchor>
      <arglist>(rcl_wait_set_t *wait_set, const std::nothrow_t &amp;) noexcept</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>interrupt_all_wait_sets</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>ac2d2b3dc9f869675621b9c53714b9f86</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; SubContext &gt;</type>
      <name>get_sub_context</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>a433b110b4d305be6b690ec3e08f7ec28</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function" protection="protected" virtualness="virtual">
      <type>virtual void</type>
      <name>clean_up</name>
      <anchorfile>classrclcpp_1_1Context.html</anchorfile>
      <anchor>ac0cafbb959c061a01dd0fe807cd31f42</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::ContextAlreadyInitialized</name>
    <filename>classrclcpp_1_1ContextAlreadyInitialized.html</filename>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>ContextAlreadyInitialized</name>
      <anchorfile>classrclcpp_1_1ContextAlreadyInitialized.html</anchorfile>
      <anchor>a99a8ae72699824a041d2c6e30fb26f28</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::contexts::DefaultContext</name>
    <filename>classrclcpp_1_1contexts_1_1DefaultContext.html</filename>
    <base>rclcpp::Context</base>
    <member kind="function">
      <type></type>
      <name>DefaultContext</name>
      <anchorfile>classrclcpp_1_1contexts_1_1DefaultContext.html</anchorfile>
      <anchor>aecc2e2c05d51693cd9b03354ccb6b535</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Duration</name>
    <filename>classrclcpp_1_1Duration.html</filename>
    <member kind="function">
      <type></type>
      <name>Duration</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>af2ab74e1b3f70436bb0451dd8a17c8e8</anchor>
      <arglist>(int32_t seconds, uint32_t nanoseconds)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Duration</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a1da7276971caca9bc6fa1ce60eab777a</anchor>
      <arglist>(rcl_duration_value_t nanoseconds)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Duration</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a383d6571ae3f50de78987065de6d7957</anchor>
      <arglist>(std::chrono::nanoseconds nanoseconds)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Duration</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>ab48a972c06e3a18fdde29268826f5491</anchor>
      <arglist>(const std::chrono::duration&lt; Rep, Period &gt; &amp;duration)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Duration</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a82c2b336ee5003391e9ca1306c209be1</anchor>
      <arglist>(const builtin_interfaces::msg::Duration &amp;duration_msg)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Duration</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a5bbb5e1f60956c7856e97e0729b10583</anchor>
      <arglist>(const rcl_duration_t &amp;duration)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Duration</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>ab34864923bd583800de30d2c5621a9fd</anchor>
      <arglist>(const Duration &amp;rhs)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Duration</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>acb312877e346bef5b4a49006c265ca64</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator builtin_interfaces::msg::Duration</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a02195a2a3f76eaa99e6e72d68db4f2a5</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Duration &amp;</type>
      <name>operator=</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a36c933fa1dabcd9ccb25c776da3bee24</anchor>
      <arglist>(const Duration &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>Duration &amp;</type>
      <name>operator=</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a030b9586430b70b95e294f1f83554436</anchor>
      <arglist>(const builtin_interfaces::msg::Duration &amp;duration_msg)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a044d4cfc47dcd899a889c83c3d4150fc</anchor>
      <arglist>(const rclcpp::Duration &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a99f58fa2793b36a88441b13395d14596</anchor>
      <arglist>(const rclcpp::Duration &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>ae03e5688a3fda25423d0bf33d898e186</anchor>
      <arglist>(const rclcpp::Duration &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;=</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>abe72e5eeec8fae6a48a49069bcfcee5e</anchor>
      <arglist>(const rclcpp::Duration &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;=</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>aec2185eac973d7b87f74926ff789d42d</anchor>
      <arglist>(const rclcpp::Duration &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>ae5d7c31b534ec70c6420ad5f238b6afe</anchor>
      <arglist>(const rclcpp::Duration &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>Duration</type>
      <name>operator+</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a76f12cff19161285a413354a6672c0a0</anchor>
      <arglist>(const rclcpp::Duration &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>Duration</type>
      <name>operator-</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>aabd15c6f4ac78b5437efdcd0f158d376</anchor>
      <arglist>(const rclcpp::Duration &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>Duration</type>
      <name>operator*</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a99768b255f9f0a55b992529dd832a3a9</anchor>
      <arglist>(double scale) const</arglist>
    </member>
    <member kind="function">
      <type>rcl_duration_value_t</type>
      <name>nanoseconds</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a044b249ddad5b98a96fad4292c011033</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>seconds</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a2294b94b1434fc532d2f153757ed4ba6</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>DurationT</type>
      <name>to_chrono</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a671704beeb2de5c6b48b4082c38a7804</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rmw_time_t</type>
      <name>to_rmw_time</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a83c263c81dafb3263f106bf7f72aa697</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static Duration</type>
      <name>max</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>ac39f968527116f2206f92db6c9652eba</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static Duration</type>
      <name>from_seconds</name>
      <anchorfile>classrclcpp_1_1Duration.html</anchorfile>
      <anchor>a39ce5aa6d0bb190d7d59857aa7a9af1a</anchor>
      <arglist>(double seconds)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::DynamicStorage</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</filename>
    <base>StoragePolicyCommon&lt; false &gt;</base>
    <class kind="class">rclcpp::wait_set_policies::DynamicStorage::SubscriptionEntry</class>
    <class kind="class">rclcpp::wait_set_policies::DynamicStorage::WaitableEntry</class>
    <class kind="class">rclcpp::wait_set_policies::DynamicStorage::WeakSubscriptionEntry</class>
    <class kind="class">rclcpp::wait_set_policies::DynamicStorage::WeakWaitableEntry</class>
    <member kind="typedef" protection="protected">
      <type>std::true_type</type>
      <name>is_mutable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a43fa163f2240a64a90dced11d05b7312</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::vector&lt; WeakSubscriptionEntry &gt;</type>
      <name>SequenceOfWeakSubscriptions</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a1d01ed9d1e3508aefcf49f14ff4fff2b</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::vector&lt; SubscriptionEntry &gt;</type>
      <name>SubscriptionsIterable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>ad43547aa5c637da98ebd58324bf47714</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::vector&lt; std::weak_ptr&lt; rclcpp::GuardCondition &gt; &gt;</type>
      <name>SequenceOfWeakGuardConditions</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a5a078255200075c6948b7344e5472568</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::vector&lt; std::shared_ptr&lt; rclcpp::GuardCondition &gt; &gt;</type>
      <name>GuardConditionsIterable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a75a76c4c5edaaec48c9fe9f7c4dfb2e4</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::vector&lt; std::weak_ptr&lt; rclcpp::TimerBase &gt; &gt;</type>
      <name>SequenceOfWeakTimers</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a3e8a5a1650087a80704bd565c3931fd0</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::vector&lt; std::shared_ptr&lt; rclcpp::TimerBase &gt; &gt;</type>
      <name>TimersIterable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a68cdae075e4688b366a4ca2f91eedb3d</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::vector&lt; std::weak_ptr&lt; rclcpp::ClientBase &gt; &gt;</type>
      <name>SequenceOfWeakClients</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a1b14d0f9d0a3ea1306cd0d71ed5e5350</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::vector&lt; std::shared_ptr&lt; rclcpp::ClientBase &gt; &gt;</type>
      <name>ClientsIterable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a3b16bccb83522246dd85452071f647e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::vector&lt; std::weak_ptr&lt; rclcpp::ServiceBase &gt; &gt;</type>
      <name>SequenceOfWeakServices</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>abc60cb5d92df21df066035b6bbb781c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::vector&lt; std::shared_ptr&lt; rclcpp::ServiceBase &gt; &gt;</type>
      <name>ServicesIterable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>ae510ae6ea7ccb5e450961bd224c81e3c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::vector&lt; WeakWaitableEntry &gt;</type>
      <name>SequenceOfWeakWaitables</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>adc83a20348246322c4a12ebccf803792</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::vector&lt; WaitableEntry &gt;</type>
      <name>WaitablesIterable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>ace8b83be17c8d116aadd9d07fe38cfda</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>DynamicStorage</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>aad1d3cd459970dd0cae88d0202272804</anchor>
      <arglist>(const SubscriptionsIterable &amp;subscriptions, const GuardConditionsIterable &amp;guard_conditions, const ArrayOfExtraGuardConditions &amp;extra_guard_conditions, const TimersIterable &amp;timers, const ClientsIterable &amp;clients, const ServicesIterable &amp;services, const WaitablesIterable &amp;waitables, rclcpp::Context::SharedPtr context)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>~DynamicStorage</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>ac4ec6bf943c9d8d30bfeecb70eb53ea9</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_rebuild_rcl_wait_set</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a89f68577969a575283d2162fc3d2ba95</anchor>
      <arglist>(const ArrayOfExtraGuardConditions &amp;extra_guard_conditions)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_add_subscription</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a21e41dab1c1f4e0832ff7acc24dcdb6c</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; &amp;&amp;subscription)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_remove_subscription</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a351b17abd01a421839e0e55e82c23962</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; &amp;&amp;subscription)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_add_guard_condition</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a6b57bc04a0e7889f634da3cbea094d7c</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::GuardCondition &gt; &amp;&amp;guard_condition)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_remove_guard_condition</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>ac30e4ffee9d0fcbc47a7701e6f837a42</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::GuardCondition &gt; &amp;&amp;guard_condition)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_add_timer</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a6f3d394d183585644b71b8f7c943269a</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::TimerBase &gt; &amp;&amp;timer)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_remove_timer</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>ace4e5a3169921af8cf2e754e9daaab97</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::TimerBase &gt; &amp;&amp;timer)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_add_client</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a2bc36721bf6532c516af877b67691410</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ClientBase &gt; &amp;&amp;client)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_remove_client</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>ac4d2951c5e72ca6c54d212b414cb85c4</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ClientBase &gt; &amp;&amp;client)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_add_service</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>aaac50d5bc12a549ec6a9446e2c433f11</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ServiceBase &gt; &amp;&amp;service)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_remove_service</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>ac50b2b32664caf5c156a8198ee4e7082</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ServiceBase &gt; &amp;&amp;service)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_add_waitable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a3ed5e1d3ce8064c0a53c46292c072169</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Waitable &gt; &amp;&amp;waitable, std::shared_ptr&lt; void &gt; &amp;&amp;associated_entity)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_remove_waitable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>aafb7665cdda58abf3a2cea11081bd27f</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Waitable &gt; &amp;&amp;waitable)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_prune_deleted_entities</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>acb96f2168db08c2a0c592f7458cf94ca</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_acquire_ownerships</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a1ca5c4b784312640e6b2c94c887942be</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_release_ownerships</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a4c28c552d1191b47a3f50bf346633462</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected" static="yes">
      <type>static bool</type>
      <name>storage_has_entity</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>ac33e369c398370d754a7a1a696a3a99c</anchor>
      <arglist>(const EntityT &amp;entity, const SequenceOfEntitiesT &amp;entities)</arglist>
    </member>
    <member kind="function" protection="protected" static="yes">
      <type>static auto</type>
      <name>storage_find_entity</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>ae70a30a6f68ab8490e8180bdf76563cf</anchor>
      <arglist>(const EntityT &amp;entity, const SequenceOfEntitiesT &amp;entities)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>size_t</type>
      <name>ownership_reference_counter_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>aa8038332abb4f48c895d58c2c516dd3e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>SequenceOfWeakSubscriptions</type>
      <name>subscriptions_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>aa181b44b799e416f0560254ed11f1832</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>SubscriptionsIterable</type>
      <name>shared_subscriptions_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a0074b5243b0ac85a3d32fe0d82a94e93</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>SequenceOfWeakGuardConditions</type>
      <name>guard_conditions_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a25957f3dda1f943dd2d4a020a5d6f434</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>GuardConditionsIterable</type>
      <name>shared_guard_conditions_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a8411cbedecb792656b70623915c120e6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>SequenceOfWeakTimers</type>
      <name>timers_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a098ed6189af21fb3860cb63344a172f2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>TimersIterable</type>
      <name>shared_timers_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a3258c41b4da46f72bb97864c7cec7862</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>SequenceOfWeakClients</type>
      <name>clients_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a1aa7d119f3c43fea4df84f7dfaab7646</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>ClientsIterable</type>
      <name>shared_clients_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a8e00730fc315a6e0717e07f3ca4a4ca1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>SequenceOfWeakServices</type>
      <name>services_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>afdbe5b0dfd7b556beafa49f2ec6c3558</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>ServicesIterable</type>
      <name>shared_services_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a65163a7c8f94eb39541f7c637c52e3b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>SequenceOfWeakWaitables</type>
      <name>waitables_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>acda871589ee50812bda47f8bec5b5ee3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>WaitablesIterable</type>
      <name>shared_waitables_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage.html</anchorfile>
      <anchor>a7c64ec61ee85f30079bf39423927aa80</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Event</name>
    <filename>classrclcpp_1_1Event.html</filename>
    <member kind="function">
      <type></type>
      <name>Event</name>
      <anchorfile>classrclcpp_1_1Event.html</anchorfile>
      <anchor>a4a6e362373a618a7ee49894aeddcda00</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>set</name>
      <anchorfile>classrclcpp_1_1Event.html</anchorfile>
      <anchor>af4db8c21cca2840f65701f6a9896e2cc</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>check</name>
      <anchorfile>classrclcpp_1_1Event.html</anchorfile>
      <anchor>a7558d375b6c950cedbad7fe39683eaac</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>check_and_clear</name>
      <anchorfile>classrclcpp_1_1Event.html</anchorfile>
      <anchor>a600da263e3c1993b45b675aaffc9633e</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::EventNotRegisteredError</name>
    <filename>classrclcpp_1_1exceptions_1_1EventNotRegisteredError.html</filename>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>EventNotRegisteredError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1EventNotRegisteredError.html</anchorfile>
      <anchor>a82c98a8fcab600bc9eb711cfb8fe77e4</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::experimental::ExecutableList</name>
    <filename>classrclcpp_1_1experimental_1_1ExecutableList.html</filename>
    <member kind="function">
      <type></type>
      <name>ExecutableList</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>a42e4af2586027738f1e20a749861c4dd</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ExecutableList</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>a32624912f74fd76e8db859a81a825a13</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>clear</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>a3c7a1efdf9ed1c882dcaaea3fa2db259</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_subscription</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>aa75c32ba90987ccf8c54da745fd8819a</anchor>
      <arglist>(rclcpp::SubscriptionBase::SharedPtr subscription)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_timer</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>a7fe2aa34b6503b6c07e20e770cd5dcee</anchor>
      <arglist>(rclcpp::TimerBase::SharedPtr timer)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_service</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>ae02588c6463e43150fe6208dfb2882b0</anchor>
      <arglist>(rclcpp::ServiceBase::SharedPtr service)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_client</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>adf40a16f6f6e67e671b6aa3e16518566</anchor>
      <arglist>(rclcpp::ClientBase::SharedPtr client)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_waitable</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>a769dad8c45916f870066568fa4f169bd</anchor>
      <arglist>(rclcpp::Waitable::SharedPtr waitable)</arglist>
    </member>
    <member kind="variable">
      <type>std::vector&lt; rclcpp::SubscriptionBase::SharedPtr &gt;</type>
      <name>subscription</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>a071a62d77c1fad0bb6ea9f4102c438a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>size_t</type>
      <name>number_of_subscriptions</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>af46df23f382d21720f7ab566607548a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::vector&lt; rclcpp::TimerBase::SharedPtr &gt;</type>
      <name>timer</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>a9ed40c959991ba00ddcd6298e31b2dd9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>size_t</type>
      <name>number_of_timers</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>a71a391de5f7c00fc3ea872c1925d8018</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::vector&lt; rclcpp::ServiceBase::SharedPtr &gt;</type>
      <name>service</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>ac6d5aa23e3c47440e90c55e368568818</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>size_t</type>
      <name>number_of_services</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>a01c7295665431cd736816a923059d364</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::vector&lt; rclcpp::ClientBase::SharedPtr &gt;</type>
      <name>client</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>a334d33e9f7b445c353281bb0429f8f8f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>size_t</type>
      <name>number_of_clients</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>aa9f70a9ff2c29b853cb737f2e043f366</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::vector&lt; rclcpp::Waitable::SharedPtr &gt;</type>
      <name>waitable</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>ae447aa59f2f1b1c04eff0662250b7ecc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>size_t</type>
      <name>number_of_waitables</name>
      <anchorfile>classrclcpp_1_1experimental_1_1ExecutableList.html</anchorfile>
      <anchor>a4045ddf9fc1735810f8c8bc9381776c8</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Executor</name>
    <filename>classrclcpp_1_1Executor.html</filename>
    <member kind="function">
      <type></type>
      <name>Executor</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a3d28d13a7233751c9b7b023528f10228</anchor>
      <arglist>(const rclcpp::ExecutorOptions &amp;options=rclcpp::ExecutorOptions())</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Executor</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a946f5bfe58cb0ee5f52a98455248ed66</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>spin</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>ad096fec6846af2169ee97cd2d23e96ca</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>add_node</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a53f9649e4c2f3593c930f4d191e7e208</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify=true)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>add_node</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a83a2b87b3a60d3fd8525d18879cec14c</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; node_ptr, bool notify=true)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>remove_node</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a660424725a58fc412fac8e86191b1f61</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify=true)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>remove_node</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>ae0543bf6a85ae11a042cef13077ab364</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; node_ptr, bool notify=true)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin_node_once</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>af09dd5b81cc174fd7029fab65a726db9</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node, std::chrono::duration&lt; RepT, T &gt; timeout=std::chrono::duration&lt; RepT, T &gt;(-1))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin_node_once</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a9791fd5812f06add439c9cca86d6226b</anchor>
      <arglist>(std::shared_ptr&lt; NodeT &gt; node, std::chrono::duration&lt; RepT, T &gt; timeout=std::chrono::duration&lt; RepT, T &gt;(-1))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin_node_some</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a49780b2217485638b0d7be720062e630</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin_node_some</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a9c8f89d67464b75cfc63f1ddf56f8850</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; node)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>spin_some</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>ae821cd4cf1040fe2da521fb19a458fa1</anchor>
      <arglist>(std::chrono::nanoseconds max_duration=std::chrono::nanoseconds(0))</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>spin_once</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>ae4abb57eea7e1b6c95acc0e7abff62b2</anchor>
      <arglist>(std::chrono::nanoseconds timeout=std::chrono::nanoseconds(-1))</arglist>
    </member>
    <member kind="function">
      <type>FutureReturnCode</type>
      <name>spin_until_future_complete</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a484f01fbbbdc2604f349586d2b2f6830</anchor>
      <arglist>(const std::shared_future&lt; ResponseT &gt; &amp;future, std::chrono::duration&lt; TimeRepT, TimeT &gt; timeout=std::chrono::duration&lt; TimeRepT, TimeT &gt;(-1))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cancel</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a0fb21996107bcf4e62f8330e8d201e70</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_memory_strategy</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>aa41ca2b0358c875337ac13a8130987e9</anchor>
      <arglist>(memory_strategy::MemoryStrategy::SharedPtr memory_strategy)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>spin_node_once_nanoseconds</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a562cd027f960258673a737c9e8d3be75</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node, std::chrono::nanoseconds timeout)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>execute_any_executable</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a915f4738cf48d6c08851be8264aad58d</anchor>
      <arglist>(AnyExecutable &amp;any_exec)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>wait_for_work</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>af211465e15ae5b13bac0893ef9e0ed4e</anchor>
      <arglist>(std::chrono::nanoseconds timeout=std::chrono::nanoseconds(-1))</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>rclcpp::node_interfaces::NodeBaseInterface::SharedPtr</type>
      <name>get_node_by_group</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>ac865aeb892b5bf2a585bcdff1ed9fe00</anchor>
      <arglist>(rclcpp::CallbackGroup::SharedPtr group)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>rclcpp::CallbackGroup::SharedPtr</type>
      <name>get_group_by_timer</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a06baded5884fc91e6d36ab676f0b7fc3</anchor>
      <arglist>(rclcpp::TimerBase::SharedPtr timer)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>bool</type>
      <name>get_next_ready_executable</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>aad3c92aec4730186388701398c84263b</anchor>
      <arglist>(AnyExecutable &amp;any_executable)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>bool</type>
      <name>get_next_executable</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a63b530d7459b579ca22044a5e7a37a50</anchor>
      <arglist>(AnyExecutable &amp;any_executable, std::chrono::nanoseconds timeout=std::chrono::nanoseconds(-1))</arglist>
    </member>
    <member kind="function" protection="protected" static="yes">
      <type>static void</type>
      <name>execute_subscription</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a3f825d7a6d22eb8a6c2974d89d8bcebc</anchor>
      <arglist>(rclcpp::SubscriptionBase::SharedPtr subscription)</arglist>
    </member>
    <member kind="function" protection="protected" static="yes">
      <type>static void</type>
      <name>execute_timer</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a6e62a3a7c60ab1569659aeaa47886b7e</anchor>
      <arglist>(rclcpp::TimerBase::SharedPtr timer)</arglist>
    </member>
    <member kind="function" protection="protected" static="yes">
      <type>static void</type>
      <name>execute_service</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a8c2e10441f4ad3c566c84f44e8d697d8</anchor>
      <arglist>(rclcpp::ServiceBase::SharedPtr service)</arglist>
    </member>
    <member kind="function" protection="protected" static="yes">
      <type>static void</type>
      <name>execute_client</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a488790ad5eccaf4a7dc2b6eab6862147</anchor>
      <arglist>(rclcpp::ClientBase::SharedPtr client)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::atomic_bool</type>
      <name>spinning</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>afe759d8451caa0752e2209bb0863dbcd</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rcl_guard_condition_t</type>
      <name>interrupt_guard_condition_</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>af08ff5546ad531eae007ce653bf1b163</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rcl_wait_set_t</type>
      <name>wait_set_</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a07bb6ff536c37f44a5c05f3ba5c300ee</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::mutex</type>
      <name>memory_strategy_mutex_</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>ab74228f8c4ef5138223eed4ed0d2559f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>memory_strategy::MemoryStrategy::SharedPtr</type>
      <name>memory_strategy_</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>aa1d31b355624a35e3a806e5533084f78</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Context &gt;</type>
      <name>context_</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a9cb729b9e8a7e925e08fb0a9d6244282</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::list&lt; rclcpp::node_interfaces::NodeBaseInterface::WeakPtr &gt;</type>
      <name>weak_nodes_</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>a83944e8c14cee34664cf40d1bc29e058</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::list&lt; const rcl_guard_condition_t * &gt;</type>
      <name>guard_conditions_</name>
      <anchorfile>classrclcpp_1_1Executor.html</anchorfile>
      <anchor>ad9be900933db041b3b72f02d043a6c5e</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::ExecutorOptions</name>
    <filename>structrclcpp_1_1ExecutorOptions.html</filename>
    <member kind="function">
      <type></type>
      <name>ExecutorOptions</name>
      <anchorfile>structrclcpp_1_1ExecutorOptions.html</anchorfile>
      <anchor>a0c3c427314664fd714e6ba2e05c644b4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::memory_strategy::MemoryStrategy::SharedPtr</type>
      <name>memory_strategy</name>
      <anchorfile>structrclcpp_1_1ExecutorOptions.html</anchorfile>
      <anchor>aa1a5a1fe4792279046677f1a4882c192</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::Context::SharedPtr</type>
      <name>context</name>
      <anchorfile>structrclcpp_1_1ExecutorOptions.html</anchorfile>
      <anchor>adf4c4b36e0e154a50755b59cd10109f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>size_t</type>
      <name>max_conditions</name>
      <anchorfile>structrclcpp_1_1ExecutorOptions.html</anchorfile>
      <anchor>a40278fb5e2c742d5e53c8d9b33745cac</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::subscription_traits::extract_message_type</name>
    <filename>structrclcpp_1_1subscription__traits_1_1extract__message__type.html</filename>
    <templarg></templarg>
    <member kind="typedef">
      <type>typename std::remove_cv&lt; MessageT &gt;::type</type>
      <name>type</name>
      <anchorfile>structrclcpp_1_1subscription__traits_1_1extract__message__type.html</anchorfile>
      <anchor>ab8bf5d6f349c72508d33ef7416f872d1</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>extract_message_type&lt; rclcpp::function_traits::function_traits&lt; CallbackT &gt;::template argument_type&lt; 0 &gt; &gt;</name>
    <filename>structrclcpp_1_1subscription__traits_1_1extract__message__type.html</filename>
    <member kind="typedef">
      <type>typename std::remove_cv&lt; rclcpp::function_traits::function_traits&lt; CallbackT &gt;::template argument_type&lt; 0 &gt; &gt;::type</type>
      <name>type</name>
      <anchorfile>structrclcpp_1_1subscription__traits_1_1extract__message__type.html</anchorfile>
      <anchor>ab8bf5d6f349c72508d33ef7416f872d1</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::subscription_traits::extract_message_type&lt; std::shared_ptr&lt; MessageT &gt; &gt;</name>
    <filename>structrclcpp_1_1subscription__traits_1_1extract__message__type_3_01std_1_1shared__ptr_3_01MessageT_01_4_01_4.html</filename>
    <templarg></templarg>
    <base>rclcpp::subscription_traits::extract_message_type</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::subscription_traits::extract_message_type&lt; std::unique_ptr&lt; MessageT, Deleter &gt; &gt;</name>
    <filename>structrclcpp_1_1subscription__traits_1_1extract__message__type_3_01std_1_1unique__ptr_3_01MessageT_00_01Deleter_01_4_01_4.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <base>rclcpp::subscription_traits::extract_message_type</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::function_traits::function_traits</name>
    <filename>structrclcpp_1_1function__traits_1_1function__traits.html</filename>
    <templarg></templarg>
    <member kind="typedef">
      <type>typename tuple_tail&lt; typename function_traits&lt; decltype(&amp;FunctionT::operator())&gt;::arguments &gt;::type</type>
      <name>arguments</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1function__traits.html</anchorfile>
      <anchor>ac4297f152c08b96057b17e8d8d711ed9</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename std::tuple_element&lt; N, arguments &gt;::type</type>
      <name>argument_type</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1function__traits.html</anchorfile>
      <anchor>ab15ff3df3dbd31898966015ddffcad2b</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename function_traits&lt; decltype(&amp;FunctionT::operator())&gt;::return_type</type>
      <name>return_type</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1function__traits.html</anchorfile>
      <anchor>a6563c2808522a4e13c5a15f398f90ad6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static constexpr std::size_t</type>
      <name>arity</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1function__traits.html</anchorfile>
      <anchor>aaf6ec8e91cff3a11fbc0d46f04bad9f2</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::function_traits::function_traits&lt; FunctionT &amp; &gt;</name>
    <filename>structrclcpp_1_1function__traits_1_1function__traits_3_01FunctionT_01_6_01_4.html</filename>
    <templarg></templarg>
    <base>rclcpp::function_traits::function_traits</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::function_traits::function_traits&lt; FunctionT &amp;&amp; &gt;</name>
    <filename>structrclcpp_1_1function__traits_1_1function__traits_3_01FunctionT_01_6_6_01_4.html</filename>
    <templarg></templarg>
    <base>rclcpp::function_traits::function_traits</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::function_traits::function_traits&lt; ReturnTypeT(*)(Args ...)&gt;</name>
    <filename>structrclcpp_1_1function__traits_1_1function__traits_3_01ReturnTypeT_07_5_08_07Args_01_8_8_8_08_4.html</filename>
    <templarg></templarg>
    <templarg>Args</templarg>
    <base>rclcpp::function_traits::function_traits&lt; ReturnTypeT(Args ...)&gt;</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::function_traits::function_traits&lt; ReturnTypeT(Args ...)&gt;</name>
    <filename>structrclcpp_1_1function__traits_1_1function__traits_3_01ReturnTypeT_07Args_01_8_8_8_08_4.html</filename>
    <templarg></templarg>
    <templarg>Args</templarg>
    <member kind="typedef">
      <type>std::tuple&lt; Args ... &gt;</type>
      <name>arguments</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1function__traits_3_01ReturnTypeT_07Args_01_8_8_8_08_4.html</anchorfile>
      <anchor>a3dcbc234577ffed31265ab9708c12d82</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename std::tuple_element&lt; N, arguments &gt;::type</type>
      <name>argument_type</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1function__traits_3_01ReturnTypeT_07Args_01_8_8_8_08_4.html</anchorfile>
      <anchor>af894bbeb4babc4b733dbf0e3f4be20b9</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>ReturnTypeT</type>
      <name>return_type</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1function__traits_3_01ReturnTypeT_07Args_01_8_8_8_08_4.html</anchorfile>
      <anchor>a98514f9b4e16231aed898be2aa3118b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static constexpr std::size_t</type>
      <name>arity</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1function__traits_3_01ReturnTypeT_07Args_01_8_8_8_08_4.html</anchorfile>
      <anchor>a1e5ed94eaf10d4adcc6833ec466fac73</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>function_traits&lt; ReturnTypeT(ClassT &amp;, Args ...)&gt;</name>
    <filename>structrclcpp_1_1function__traits_1_1function__traits.html</filename>
    <member kind="typedef">
      <type>typename tuple_tail&lt; typename function_traits&lt; decltype(&amp;ReturnTypeT(ClassT &amp;, Args ...) ::operator())&gt;::arguments &gt;::type</type>
      <name>arguments</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1function__traits.html</anchorfile>
      <anchor>ac4297f152c08b96057b17e8d8d711ed9</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename std::tuple_element&lt; N, arguments &gt;::type</type>
      <name>argument_type</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1function__traits.html</anchorfile>
      <anchor>ab15ff3df3dbd31898966015ddffcad2b</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename function_traits&lt; decltype(&amp;ReturnTypeT(ClassT &amp;, Args ...) ::operator())&gt;::return_type</type>
      <name>return_type</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1function__traits.html</anchorfile>
      <anchor>a6563c2808522a4e13c5a15f398f90ad6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static constexpr std::size_t</type>
      <name>arity</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1function__traits.html</anchorfile>
      <anchor>aaf6ec8e91cff3a11fbc0d46f04bad9f2</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::function_traits::function_traits&lt; ReturnTypeT(ClassT::*)(Args ...) const &gt;</name>
    <filename>structrclcpp_1_1function__traits_1_1function__traits_3_01ReturnTypeT_07ClassT_1_1_5_08_07Args_01_8_8_8_08_01const_01_4.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <templarg>Args</templarg>
    <base>function_traits&lt; ReturnTypeT(ClassT &amp;, Args ...)&gt;</base>
  </compound>
  <compound kind="class">
    <name>rclcpp::GenericRate</name>
    <filename>classrclcpp_1_1GenericRate.html</filename>
    <templarg></templarg>
    <base>rclcpp::RateBase</base>
    <member kind="function">
      <type></type>
      <name>GenericRate</name>
      <anchorfile>classrclcpp_1_1GenericRate.html</anchorfile>
      <anchor>aff738ca0ffbd04d34f663d90724af1e3</anchor>
      <arglist>(double rate)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>GenericRate</name>
      <anchorfile>classrclcpp_1_1GenericRate.html</anchorfile>
      <anchor>aef242e542d5f799b998d85425aa5c36f</anchor>
      <arglist>(std::chrono::nanoseconds period)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>sleep</name>
      <anchorfile>classrclcpp_1_1GenericRate.html</anchorfile>
      <anchor>a24810c63ec3fc0d5a9d873a654c56e9b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>is_steady</name>
      <anchorfile>classrclcpp_1_1GenericRate.html</anchorfile>
      <anchor>a4440aac47b8b09fbd0f3c7b002591634</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>reset</name>
      <anchorfile>classrclcpp_1_1GenericRate.html</anchorfile>
      <anchor>af865b167266c1c36ad45aa5f4fd363b2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::chrono::nanoseconds</type>
      <name>period</name>
      <anchorfile>classrclcpp_1_1GenericRate.html</anchorfile>
      <anchor>a51a6d5e21037f9d966cb235761b624bd</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::GenericTimer</name>
    <filename>classrclcpp_1_1GenericTimer.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <base>rclcpp::TimerBase</base>
    <member kind="function">
      <type></type>
      <name>GenericTimer</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>adffcb8021ac5bb37bbde165a273af41b</anchor>
      <arglist>(Clock::SharedPtr clock, std::chrono::nanoseconds period, FunctorT &amp;&amp;callback, rclcpp::Context::SharedPtr context)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~GenericTimer</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>a6a1ec3c0ffb8f5477d1b81e6239c7af6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>execute_callback</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>a84339e6ba639b3cf71bcf48449e4dd49</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>execute_callback_delegate</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>a1a887fbad758e498bdd9e591103e6047</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>execute_callback_delegate</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>a1a887fbad758e498bdd9e591103e6047</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_steady</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>a5d4e0c19ce093680a92078e075ec49f4</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>FunctorT</type>
      <name>callback_</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>afc33cc457a1b4cfd371061d482fbbec2</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>GenericTimer&lt; FunctorT &gt;</name>
    <filename>classrclcpp_1_1GenericTimer.html</filename>
    <base>rclcpp::TimerBase</base>
    <member kind="function">
      <type></type>
      <name>GenericTimer</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>adffcb8021ac5bb37bbde165a273af41b</anchor>
      <arglist>(Clock::SharedPtr clock, std::chrono::nanoseconds period, FunctorT &amp;&amp;callback, rclcpp::Context::SharedPtr context)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~GenericTimer</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>a6a1ec3c0ffb8f5477d1b81e6239c7af6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>execute_callback</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>a84339e6ba639b3cf71bcf48449e4dd49</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>execute_callback_delegate</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>a1a887fbad758e498bdd9e591103e6047</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>execute_callback_delegate</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>a1a887fbad758e498bdd9e591103e6047</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_steady</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>a5d4e0c19ce093680a92078e075ec49f4</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>FunctorT</type>
      <name>callback_</name>
      <anchorfile>classrclcpp_1_1GenericTimer.html</anchorfile>
      <anchor>afc33cc457a1b4cfd371061d482fbbec2</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::graph_listener::GraphListener</name>
    <filename>classrclcpp_1_1graph__listener_1_1GraphListener.html</filename>
    <base>enable_shared_from_this&lt; GraphListener &gt;</base>
    <member kind="function">
      <type></type>
      <name>GraphListener</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1GraphListener.html</anchorfile>
      <anchor>ae9524713ade22eabdd065d3d76530a62</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Context &gt; parent_context)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~GraphListener</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1GraphListener.html</anchorfile>
      <anchor>a6cc8ae9c8cb188906738e0a65d8e76aa</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>start_if_not_started</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1GraphListener.html</anchorfile>
      <anchor>a2cc9078f363ba07a7c56ae6ad6a0e22c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>add_node</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1GraphListener.html</anchorfile>
      <anchor>ab9bd0af0fee4d3ee30b6a9e580c110f1</anchor>
      <arglist>(rclcpp::node_interfaces::NodeGraphInterface *node_graph)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>has_node</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1GraphListener.html</anchorfile>
      <anchor>a9bae51cbc5d0f49934f14f5a8b49914b</anchor>
      <arglist>(rclcpp::node_interfaces::NodeGraphInterface *node_graph)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>remove_node</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1GraphListener.html</anchorfile>
      <anchor>ab862c01311dfa58aac607aa317d354ab</anchor>
      <arglist>(rclcpp::node_interfaces::NodeGraphInterface *node_graph)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>shutdown</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1GraphListener.html</anchorfile>
      <anchor>a18acae41af02f0670238d32ccb9f5c9a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>shutdown</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1GraphListener.html</anchorfile>
      <anchor>a5615cd28d2f7dbc1859cadc318adb0f9</anchor>
      <arglist>(const std::nothrow_t &amp;) noexcept</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>is_shutdown</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1GraphListener.html</anchorfile>
      <anchor>ad7097e2081bd2b9bffe5010f87edf7a7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected" virtualness="virtual">
      <type>virtual void</type>
      <name>run</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1GraphListener.html</anchorfile>
      <anchor>aeb5c49d98a67d3ef49e3b1edc43e09dd</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected" virtualness="virtual">
      <type>virtual void</type>
      <name>run_loop</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1GraphListener.html</anchorfile>
      <anchor>a5f8379b6659740a29873ee413229b808</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::graph_listener::GraphListenerShutdownError</name>
    <filename>classrclcpp_1_1graph__listener_1_1GraphListenerShutdownError.html</filename>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>GraphListenerShutdownError</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1GraphListenerShutdownError.html</anchorfile>
      <anchor>a4d457a7036bec7003aef60d0e54ee269</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::GuardCondition</name>
    <filename>classrclcpp_1_1GuardCondition.html</filename>
    <member kind="function">
      <type></type>
      <name>GuardCondition</name>
      <anchorfile>classrclcpp_1_1GuardCondition.html</anchorfile>
      <anchor>a1f47c3f291a618cc5f8163ff280bc45a</anchor>
      <arglist>(rclcpp::Context::SharedPtr context=rclcpp::contexts::get_global_default_context())</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~GuardCondition</name>
      <anchorfile>classrclcpp_1_1GuardCondition.html</anchorfile>
      <anchor>ab80eba18f1f6df53215fb96000c1ccf4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Context::SharedPtr</type>
      <name>get_context</name>
      <anchorfile>classrclcpp_1_1GuardCondition.html</anchorfile>
      <anchor>a74841fb3375c383bdcf801608935b003</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const rcl_guard_condition_t &amp;</type>
      <name>get_rcl_guard_condition</name>
      <anchorfile>classrclcpp_1_1GuardCondition.html</anchorfile>
      <anchor>abdc420af4d4869de5c557df5caea2c81</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>trigger</name>
      <anchorfile>classrclcpp_1_1GuardCondition.html</anchorfile>
      <anchor>a0cf65d665f4438133ea9fa09bf556069</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>exchange_in_use_by_wait_set_state</name>
      <anchorfile>classrclcpp_1_1GuardCondition.html</anchorfile>
      <anchor>a7373921fb61a973c1bc16b7f8a3b97b6</anchor>
      <arglist>(bool in_use_state)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Context::SharedPtr</type>
      <name>context_</name>
      <anchorfile>classrclcpp_1_1GuardCondition.html</anchorfile>
      <anchor>ab348c37692544b200b194e641fb21530</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rcl_guard_condition_t</type>
      <name>rcl_guard_condition_</name>
      <anchorfile>classrclcpp_1_1GuardCondition.html</anchorfile>
      <anchor>a2963b8d379f7072953d00ad323c78678</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::atomic&lt; bool &gt;</type>
      <name>in_use_by_wait_set_</name>
      <anchorfile>classrclcpp_1_1GuardCondition.html</anchorfile>
      <anchor>ab562d0e40a96b0441e7ebf3a15a7139b</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::subscription_traits::has_message_type</name>
    <filename>structrclcpp_1_1subscription__traits_1_1has__message__type.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <base>extract_message_type&lt; rclcpp::function_traits::function_traits&lt; CallbackT &gt;::template argument_type&lt; 0 &gt; &gt;</base>
  </compound>
  <compound kind="class">
    <name>rclcpp::detail::MutexTwoPriorities::HighPriorityLockable</name>
    <filename>classrclcpp_1_1detail_1_1MutexTwoPriorities_1_1HighPriorityLockable.html</filename>
    <member kind="function">
      <type></type>
      <name>HighPriorityLockable</name>
      <anchorfile>classrclcpp_1_1detail_1_1MutexTwoPriorities_1_1HighPriorityLockable.html</anchorfile>
      <anchor>a8f1b8f3497a71947645179bdff088e14</anchor>
      <arglist>(MutexTwoPriorities &amp;parent)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>lock</name>
      <anchorfile>classrclcpp_1_1detail_1_1MutexTwoPriorities_1_1HighPriorityLockable.html</anchorfile>
      <anchor>ac5a7a0833fae850853be36c981b57708</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>unlock</name>
      <anchorfile>classrclcpp_1_1detail_1_1MutexTwoPriorities_1_1HighPriorityLockable.html</anchorfile>
      <anchor>a25d74cb5c95d043be811651c3ee85a41</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::InitOptions</name>
    <filename>classrclcpp_1_1InitOptions.html</filename>
    <member kind="function">
      <type></type>
      <name>InitOptions</name>
      <anchorfile>classrclcpp_1_1InitOptions.html</anchorfile>
      <anchor>a8c499c99605e08d779df9e787c61bca0</anchor>
      <arglist>(rcl_allocator_t allocator=rcl_get_default_allocator())</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>InitOptions</name>
      <anchorfile>classrclcpp_1_1InitOptions.html</anchorfile>
      <anchor>a855dde7f38aba0dc136a17f01fa476e7</anchor>
      <arglist>(const rcl_init_options_t &amp;init_options)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>InitOptions</name>
      <anchorfile>classrclcpp_1_1InitOptions.html</anchorfile>
      <anchor>a25072f29811bd7c64bdb3d049b011cd9</anchor>
      <arglist>(const InitOptions &amp;other)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>auto_initialize_logging</name>
      <anchorfile>classrclcpp_1_1InitOptions.html</anchorfile>
      <anchor>a078d0f36a02249b135842f603fb92da3</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>InitOptions &amp;</type>
      <name>auto_initialize_logging</name>
      <anchorfile>classrclcpp_1_1InitOptions.html</anchorfile>
      <anchor>a1c959d1998034e8ee4e00d0d01793cd1</anchor>
      <arglist>(bool initialize_logging)</arglist>
    </member>
    <member kind="function">
      <type>InitOptions &amp;</type>
      <name>operator=</name>
      <anchorfile>classrclcpp_1_1InitOptions.html</anchorfile>
      <anchor>a2261e0f7fd90e45909957d344b135070</anchor>
      <arglist>(const InitOptions &amp;other)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~InitOptions</name>
      <anchorfile>classrclcpp_1_1InitOptions.html</anchorfile>
      <anchor>ac9af088145cc20179cb0ed19580becda</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rcl_init_options_t *</type>
      <name>get_rcl_init_options</name>
      <anchorfile>classrclcpp_1_1InitOptions.html</anchorfile>
      <anchor>ad028cbb7d02ba9f4707f9cdbc8b4ce54</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>shutdown_on_sigint</name>
      <anchorfile>classrclcpp_1_1InitOptions.html</anchorfile>
      <anchor>a1243d2aeb64593534b68afe5017c7047</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>finalize_init_options</name>
      <anchorfile>classrclcpp_1_1InitOptions.html</anchorfile>
      <anchor>ade615ac330c30a51508e3eaa7031555b</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::experimental::buffers::IntraProcessBuffer</name>
    <filename>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <base>rclcpp::experimental::buffers::IntraProcessBufferBase</base>
    <member kind="typedef">
      <type>std::unique_ptr&lt; MessageT, MessageDeleter &gt;</type>
      <name>MessageUniquePtr</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>a7a29a4bf396aef54e6c8e84c94167dbd</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; const MessageT &gt;</type>
      <name>MessageSharedPtr</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>ae0060e15186826279fe9e5dd7977b59d</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~IntraProcessBuffer</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>adb36c2e0cb99e6c8d5f7fc465d236f1a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>add_shared</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>a303e862ac97d6b07a0c14ff9f3eb45f2</anchor>
      <arglist>(MessageSharedPtr msg)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>add_unique</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>ae372695798488158eb57846a3ff6b47f</anchor>
      <arglist>(MessageUniquePtr msg)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual MessageSharedPtr</type>
      <name>consume_shared</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>ad0d4f8a75c616385ea11f6f8346b6397</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual MessageUniquePtr</type>
      <name>consume_unique</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>af17fca694ccfc70d5582119605e077a2</anchor>
      <arglist>()=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>IntraProcessBuffer&lt; MessageT, std::allocator&lt; void &gt;, std::default_delete&lt; MessageT &gt; &gt;</name>
    <filename>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</filename>
    <base>rclcpp::experimental::buffers::IntraProcessBufferBase</base>
    <member kind="typedef">
      <type>std::unique_ptr&lt; MessageT, std::default_delete&lt; MessageT &gt; &gt;</type>
      <name>MessageUniquePtr</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>a7a29a4bf396aef54e6c8e84c94167dbd</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; const MessageT &gt;</type>
      <name>MessageSharedPtr</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>ae0060e15186826279fe9e5dd7977b59d</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~IntraProcessBuffer</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>adb36c2e0cb99e6c8d5f7fc465d236f1a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>add_shared</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>a303e862ac97d6b07a0c14ff9f3eb45f2</anchor>
      <arglist>(MessageSharedPtr msg)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>add_unique</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>ae372695798488158eb57846a3ff6b47f</anchor>
      <arglist>(MessageUniquePtr msg)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual MessageSharedPtr</type>
      <name>consume_shared</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>ad0d4f8a75c616385ea11f6f8346b6397</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual MessageUniquePtr</type>
      <name>consume_unique</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBuffer.html</anchorfile>
      <anchor>af17fca694ccfc70d5582119605e077a2</anchor>
      <arglist>()=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::experimental::buffers::IntraProcessBufferBase</name>
    <filename>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBufferBase.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~IntraProcessBufferBase</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBufferBase.html</anchorfile>
      <anchor>a4f0cf01b84e5d5a44e8d860f36e44706</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>clear</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBufferBase.html</anchorfile>
      <anchor>afd327f60ae17b10c4e957e6eec4673ea</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>has_data</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBufferBase.html</anchorfile>
      <anchor>a3751f4bc02aef3c36a19b9e251aa0363</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>use_take_shared_method</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1IntraProcessBufferBase.html</anchorfile>
      <anchor>a453d24c4943ffc763ed68c8e9c8cf949</anchor>
      <arglist>() const =0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::experimental::IntraProcessManager</name>
    <filename>classrclcpp_1_1experimental_1_1IntraProcessManager.html</filename>
    <member kind="function">
      <type></type>
      <name>IntraProcessManager</name>
      <anchorfile>classrclcpp_1_1experimental_1_1IntraProcessManager.html</anchorfile>
      <anchor>a1506e88c48734d99344995c697c114a4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~IntraProcessManager</name>
      <anchorfile>classrclcpp_1_1experimental_1_1IntraProcessManager.html</anchorfile>
      <anchor>a8473a1fb03c114916ed8a3db87527d0d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>uint64_t</type>
      <name>add_subscription</name>
      <anchorfile>classrclcpp_1_1experimental_1_1IntraProcessManager.html</anchorfile>
      <anchor>a26cb3ff5e62b939e328ae04fd5351868</anchor>
      <arglist>(rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr subscription)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_subscription</name>
      <anchorfile>classrclcpp_1_1experimental_1_1IntraProcessManager.html</anchorfile>
      <anchor>a366f7003b5c73cf3754a30d853484d95</anchor>
      <arglist>(uint64_t intra_process_subscription_id)</arglist>
    </member>
    <member kind="function">
      <type>uint64_t</type>
      <name>add_publisher</name>
      <anchorfile>classrclcpp_1_1experimental_1_1IntraProcessManager.html</anchorfile>
      <anchor>a49b95825a7f456802503cc4268d01440</anchor>
      <arglist>(rclcpp::PublisherBase::SharedPtr publisher)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_publisher</name>
      <anchorfile>classrclcpp_1_1experimental_1_1IntraProcessManager.html</anchorfile>
      <anchor>a694adea2d5ebbb32c6dc302f4f210f41</anchor>
      <arglist>(uint64_t intra_process_publisher_id)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>do_intra_process_publish</name>
      <anchorfile>classrclcpp_1_1experimental_1_1IntraProcessManager.html</anchorfile>
      <anchor>a1e7f2e69ee65b491e9ae64908375ecbc</anchor>
      <arglist>(uint64_t intra_process_publisher_id, std::unique_ptr&lt; MessageT, Deleter &gt; message, std::shared_ptr&lt; typename allocator::AllocRebind&lt; MessageT, Alloc &gt;::allocator_type &gt; allocator)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; const MessageT &gt;</type>
      <name>do_intra_process_publish_and_return_shared</name>
      <anchorfile>classrclcpp_1_1experimental_1_1IntraProcessManager.html</anchorfile>
      <anchor>a8f65275da6fd8ae350d6a48f9f13f5a7</anchor>
      <arglist>(uint64_t intra_process_publisher_id, std::unique_ptr&lt; MessageT, Deleter &gt; message, std::shared_ptr&lt; typename allocator::AllocRebind&lt; MessageT, Alloc &gt;::allocator_type &gt; allocator)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>matches_any_publishers</name>
      <anchorfile>classrclcpp_1_1experimental_1_1IntraProcessManager.html</anchorfile>
      <anchor>a2001cd2c7b0db22e3745c805373e41cf</anchor>
      <arglist>(const rmw_gid_t *id) const</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_subscription_count</name>
      <anchorfile>classrclcpp_1_1experimental_1_1IntraProcessManager.html</anchorfile>
      <anchor>aae9e389f064f12273ff6cc16995a738d</anchor>
      <arglist>(uint64_t intra_process_publisher_id) const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr</type>
      <name>get_subscription_intra_process</name>
      <anchorfile>classrclcpp_1_1experimental_1_1IntraProcessManager.html</anchorfile>
      <anchor>ac4852a48d8708a5f7441d4d289ec10c7</anchor>
      <arglist>(uint64_t intra_process_subscription_id)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::InvalidEventError</name>
    <filename>classrclcpp_1_1exceptions_1_1InvalidEventError.html</filename>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>InvalidEventError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1InvalidEventError.html</anchorfile>
      <anchor>ace127c415d315e495c07f577f61531ac</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::InvalidNamespaceError</name>
    <filename>classrclcpp_1_1exceptions_1_1InvalidNamespaceError.html</filename>
    <base>rclcpp::exceptions::NameValidationError</base>
    <member kind="function">
      <type></type>
      <name>InvalidNamespaceError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1InvalidNamespaceError.html</anchorfile>
      <anchor>a1652766f34ace0dfa60f335abda7c693</anchor>
      <arglist>(const char *namespace_, const char *error_msg, size_t invalid_index)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::InvalidNodeError</name>
    <filename>classrclcpp_1_1exceptions_1_1InvalidNodeError.html</filename>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>InvalidNodeError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1InvalidNodeError.html</anchorfile>
      <anchor>a160ed546618a1185404300d8c3c57f70</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::InvalidNodeNameError</name>
    <filename>classrclcpp_1_1exceptions_1_1InvalidNodeNameError.html</filename>
    <base>rclcpp::exceptions::NameValidationError</base>
    <member kind="function">
      <type></type>
      <name>InvalidNodeNameError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1InvalidNodeNameError.html</anchorfile>
      <anchor>ade7d797bfbeaa589f13e613b1ac9bd07</anchor>
      <arglist>(const char *node_name, const char *error_msg, size_t invalid_index)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::InvalidParametersException</name>
    <filename>classrclcpp_1_1exceptions_1_1InvalidParametersException.html</filename>
    <base>std::runtime_error</base>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::InvalidParameterTypeException</name>
    <filename>classrclcpp_1_1exceptions_1_1InvalidParameterTypeException.html</filename>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>InvalidParameterTypeException</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1InvalidParameterTypeException.html</anchorfile>
      <anchor>affd82d095567f089977b1e5c9dc9d03e</anchor>
      <arglist>(const std::string &amp;name, const std::string message)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::InvalidParameterValueException</name>
    <filename>classrclcpp_1_1exceptions_1_1InvalidParameterValueException.html</filename>
    <base>std::runtime_error</base>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::InvalidServiceNameError</name>
    <filename>classrclcpp_1_1exceptions_1_1InvalidServiceNameError.html</filename>
    <base>rclcpp::exceptions::NameValidationError</base>
    <member kind="function">
      <type></type>
      <name>InvalidServiceNameError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1InvalidServiceNameError.html</anchorfile>
      <anchor>a01d218433d841aff83a191ec82cf5a5d</anchor>
      <arglist>(const char *namespace_, const char *error_msg, size_t invalid_index)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::InvalidTopicNameError</name>
    <filename>classrclcpp_1_1exceptions_1_1InvalidTopicNameError.html</filename>
    <base>rclcpp::exceptions::NameValidationError</base>
    <member kind="function">
      <type></type>
      <name>InvalidTopicNameError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1InvalidTopicNameError.html</anchorfile>
      <anchor>ac3662a23741845589952d9b67c409d4d</anchor>
      <arglist>(const char *namespace_, const char *error_msg, size_t invalid_index)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::subscription_traits::is_serialized_callback</name>
    <filename>structrclcpp_1_1subscription__traits_1_1is__serialized__callback.html</filename>
    <templarg></templarg>
    <base>is_serialized_subscription_argument&lt; rclcpp::function_traits::function_traits&lt; CallbackT &gt;::template argument_type&lt; 0 &gt; &gt;</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::serialization_traits::is_serialized_message_class</name>
    <filename>structrclcpp_1_1serialization__traits_1_1is__serialized__message__class.html</filename>
    <templarg></templarg>
    <base>std::false_type</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::serialization_traits::is_serialized_message_class&lt; SerializedMessage &gt;</name>
    <filename>structrclcpp_1_1serialization__traits_1_1is__serialized__message__class_3_01SerializedMessage_01_4.html</filename>
    <base>std::true_type</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::subscription_traits::is_serialized_subscription</name>
    <filename>structrclcpp_1_1subscription__traits_1_1is__serialized__subscription.html</filename>
    <templarg></templarg>
    <base>rclcpp::subscription_traits::is_serialized_subscription_argument</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::subscription_traits::is_serialized_subscription_argument</name>
    <filename>structrclcpp_1_1subscription__traits_1_1is__serialized__subscription__argument.html</filename>
    <templarg></templarg>
    <base>std::false_type</base>
  </compound>
  <compound kind="class">
    <name>is_serialized_subscription_argument&lt; rclcpp::function_traits::function_traits&lt; CallbackT &gt;::template argument_type&lt; 0 &gt; &gt;</name>
    <filename>structrclcpp_1_1subscription__traits_1_1is__serialized__subscription__argument.html</filename>
    <base>std::false_type</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::subscription_traits::is_serialized_subscription_argument&lt; SerializedMessage &gt;</name>
    <filename>structrclcpp_1_1subscription__traits_1_1is__serialized__subscription__argument_3_01SerializedMessage_01_4.html</filename>
    <base>std::true_type</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::subscription_traits::is_serialized_subscription_argument&lt; std::shared_ptr&lt; SerializedMessage &gt; &gt;</name>
    <filename>structrclcpp_1_1subscription__traits_1_1is__serialized__subscription__argument_3_01std_1_1shared2bb0fb927f725533e666e6cdeb44c7a5.html</filename>
    <base>std::true_type</base>
  </compound>
  <compound kind="class">
    <name>rclcpp::JumpHandler</name>
    <filename>classrclcpp_1_1JumpHandler.html</filename>
    <member kind="typedef">
      <type>std::function&lt; void()&gt;</type>
      <name>pre_callback_t</name>
      <anchorfile>classrclcpp_1_1JumpHandler.html</anchorfile>
      <anchor>a1a284684ffc85ed34c858a6d2884f782</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(const rcl_time_jump_t &amp;)&gt;</type>
      <name>post_callback_t</name>
      <anchorfile>classrclcpp_1_1JumpHandler.html</anchorfile>
      <anchor>a39cc46c31ebfb27ed0e018aee2c6bda9</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>JumpHandler</name>
      <anchorfile>classrclcpp_1_1JumpHandler.html</anchorfile>
      <anchor>a69f5bd5447e1cb782a73b20ede049133</anchor>
      <arglist>(pre_callback_t pre_callback, post_callback_t post_callback, const rcl_jump_threshold_t &amp;threshold)</arglist>
    </member>
    <member kind="variable">
      <type>pre_callback_t</type>
      <name>pre_callback</name>
      <anchorfile>classrclcpp_1_1JumpHandler.html</anchorfile>
      <anchor>ab53dc82c74252e85e6210aee2733c0a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>post_callback_t</type>
      <name>post_callback</name>
      <anchorfile>classrclcpp_1_1JumpHandler.html</anchorfile>
      <anchor>a518bfd4ea80de39a445237e4f27a3a52</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rcl_jump_threshold_t</type>
      <name>notice_threshold</name>
      <anchorfile>classrclcpp_1_1JumpHandler.html</anchorfile>
      <anchor>aadf1c76a5de51a28f63be43201030ce8</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::KeepAll</name>
    <filename>structrclcpp_1_1KeepAll.html</filename>
    <base>rclcpp::QoSInitialization</base>
    <member kind="function">
      <type></type>
      <name>KeepAll</name>
      <anchorfile>structrclcpp_1_1KeepAll.html</anchorfile>
      <anchor>a76348198187df9d3cffb8f77415cf153</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::KeepLast</name>
    <filename>structrclcpp_1_1KeepLast.html</filename>
    <base>rclcpp::QoSInitialization</base>
    <member kind="function">
      <type></type>
      <name>KeepLast</name>
      <anchorfile>structrclcpp_1_1KeepLast.html</anchorfile>
      <anchor>a7f317507429cae7c87cf7679123f848c</anchor>
      <arglist>(size_t depth)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::LoanedMessage</name>
    <filename>classrclcpp_1_1LoanedMessage.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <member kind="function">
      <type></type>
      <name>LoanedMessage</name>
      <anchorfile>classrclcpp_1_1LoanedMessage.html</anchorfile>
      <anchor>aabe234aba050793443ad4230eb54096c</anchor>
      <arglist>(const rclcpp::PublisherBase &amp;pub, std::allocator&lt; MessageT &gt; allocator)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>LoanedMessage</name>
      <anchorfile>classrclcpp_1_1LoanedMessage.html</anchorfile>
      <anchor>aa1228b8751624c7ce02069f3a7f9e3ef</anchor>
      <arglist>(const rclcpp::PublisherBase *pub, std::shared_ptr&lt; std::allocator&lt; MessageT &gt;&gt; allocator)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>LoanedMessage</name>
      <anchorfile>classrclcpp_1_1LoanedMessage.html</anchorfile>
      <anchor>ab3d0474a606b692d8a2239644ae19c8c</anchor>
      <arglist>(LoanedMessage&lt; MessageT &gt; &amp;&amp;other)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~LoanedMessage</name>
      <anchorfile>classrclcpp_1_1LoanedMessage.html</anchorfile>
      <anchor>ab7eb5f8d7da5a1d67accc0c8881634c0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_valid</name>
      <anchorfile>classrclcpp_1_1LoanedMessage.html</anchorfile>
      <anchor>af5824c1d8084d72ca02fb1b663e6c3e8</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>MessageT &amp;</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1LoanedMessage.html</anchorfile>
      <anchor>aa5306552effe743ea0be81dc839c4998</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>MessageT *</type>
      <name>release</name>
      <anchorfile>classrclcpp_1_1LoanedMessage.html</anchorfile>
      <anchor>ad18fb98aa01102eae5032ae35522902d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>LoanedMessage</name>
      <anchorfile>classrclcpp_1_1LoanedMessage.html</anchorfile>
      <anchor>a84374520df3c17f9d6d9f3bef8624fa4</anchor>
      <arglist>(const LoanedMessage&lt; MessageT &gt; &amp;other)=delete</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const rclcpp::PublisherBase &amp;</type>
      <name>pub_</name>
      <anchorfile>classrclcpp_1_1LoanedMessage.html</anchorfile>
      <anchor>a3a42f521e4ce0547e5b3f62fe599ea3a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>MessageT *</type>
      <name>message_</name>
      <anchorfile>classrclcpp_1_1LoanedMessage.html</anchorfile>
      <anchor>a84995d4ebba382e1370c85c37b18e325</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>MessageAllocator</type>
      <name>message_allocator_</name>
      <anchorfile>classrclcpp_1_1LoanedMessage.html</anchorfile>
      <anchor>ae5d646964a3964c86162e90508515239</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Logger</name>
    <filename>classrclcpp_1_1Logger.html</filename>
    <member kind="function">
      <type></type>
      <name>Logger</name>
      <anchorfile>classrclcpp_1_1Logger.html</anchorfile>
      <anchor>ae472f5f6998f0a55ac36818434dbad07</anchor>
      <arglist>(const Logger &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_name</name>
      <anchorfile>classrclcpp_1_1Logger.html</anchorfile>
      <anchor>adde4321457b5d66c359aa43e1ad2d13d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Logger</type>
      <name>get_child</name>
      <anchorfile>classrclcpp_1_1Logger.html</anchorfile>
      <anchor>a68ab28459973c1072d3ad7a593ac994e</anchor>
      <arglist>(const std::string &amp;suffix)</arglist>
    </member>
    <member kind="friend" protection="private">
      <type>friend Logger</type>
      <name>rclcpp::get_logger</name>
      <anchorfile>classrclcpp_1_1Logger.html</anchorfile>
      <anchor>a065ae21b8b626fa98be4ad3ce03fb070</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::detail::MutexTwoPriorities::LowPriorityLockable</name>
    <filename>classrclcpp_1_1detail_1_1MutexTwoPriorities_1_1LowPriorityLockable.html</filename>
    <member kind="function">
      <type></type>
      <name>LowPriorityLockable</name>
      <anchorfile>classrclcpp_1_1detail_1_1MutexTwoPriorities_1_1LowPriorityLockable.html</anchorfile>
      <anchor>a52074f4096c396f5d61595fc49e2cadf</anchor>
      <arglist>(MutexTwoPriorities &amp;parent)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>lock</name>
      <anchorfile>classrclcpp_1_1detail_1_1MutexTwoPriorities_1_1LowPriorityLockable.html</anchorfile>
      <anchor>a284cb072cd19984006ac76ceb13c6274</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>unlock</name>
      <anchorfile>classrclcpp_1_1detail_1_1MutexTwoPriorities_1_1LowPriorityLockable.html</anchorfile>
      <anchor>a6fc68919b214957385fad43a80f18cf5</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::memory_strategy::MemoryStrategy</name>
    <filename>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</filename>
    <member kind="typedef">
      <type>std::list&lt; rclcpp::node_interfaces::NodeBaseInterface::WeakPtr &gt;</type>
      <name>WeakNodeList</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>af517c4e1fe697380a6d61b47d30ff3f9</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~MemoryStrategy</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a4e5d1d06868e7024001360486608e0b3</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>collect_entities</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a68d1190bef11fd3f191833ff203f5fdc</anchor>
      <arglist>(const WeakNodeList &amp;weak_nodes)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual size_t</type>
      <name>number_of_ready_subscriptions</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>ada06c00c2b2fb96dc71a947b1d0acc30</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual size_t</type>
      <name>number_of_ready_services</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>ad66956f123dd61853b44c0b4c273b8be</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual size_t</type>
      <name>number_of_ready_clients</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>adac34b3efd3b3e1af3c5ce56e93436cc</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual size_t</type>
      <name>number_of_ready_events</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a84ac70807d13d8d19eebfdcafad3f99e</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual size_t</type>
      <name>number_of_ready_timers</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a317b2a13154740338c82f14f8ad17f35</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual size_t</type>
      <name>number_of_guard_conditions</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>afa9a7fa1485eb0f96f4e235192116cb6</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual size_t</type>
      <name>number_of_waitables</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a1b084b4c0e52dd3b8061edc591cf1846</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>add_waitable_handle</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>abf14e43aea3c29cff915a00ac356d16b</anchor>
      <arglist>(const rclcpp::Waitable::SharedPtr &amp;waitable)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>add_handles_to_wait_set</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>af998f1defc68555fc6ca4a846352dcdd</anchor>
      <arglist>(rcl_wait_set_t *wait_set)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>clear_handles</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a8f1d55381e4cc946d6f7ea69cc4cc40e</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>remove_null_handles</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a5400c3576b8ff01350944518204ac585</anchor>
      <arglist>(rcl_wait_set_t *wait_set)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>add_guard_condition</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a397f7d368609d419fbabc6e89b21185a</anchor>
      <arglist>(const rcl_guard_condition_t *guard_condition)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>remove_guard_condition</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a3d89cd7026895cf46c04df528536d40c</anchor>
      <arglist>(const rcl_guard_condition_t *guard_condition)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>get_next_subscription</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a013baaa31920745192b7348b7061a603</anchor>
      <arglist>(rclcpp::AnyExecutable &amp;any_exec, const WeakNodeList &amp;weak_nodes)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>get_next_service</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a60376295dc3ffc92cf71b4d3f2d1dde6</anchor>
      <arglist>(rclcpp::AnyExecutable &amp;any_exec, const WeakNodeList &amp;weak_nodes)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>get_next_client</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a4af883f9d198fa4562be6eac3d09dca7</anchor>
      <arglist>(rclcpp::AnyExecutable &amp;any_exec, const WeakNodeList &amp;weak_nodes)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>get_next_timer</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a5e5c389bd4fd76796c8562c4dea27842</anchor>
      <arglist>(rclcpp::AnyExecutable &amp;any_exec, const WeakNodeList &amp;weak_nodes)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>get_next_waitable</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a1a731571a238adba861b48657d1b58ef</anchor>
      <arglist>(rclcpp::AnyExecutable &amp;any_exec, const WeakNodeList &amp;weak_nodes)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rcl_allocator_t</type>
      <name>get_allocator</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a039030d304fcd3f40b1d871850f24a41</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static rclcpp::SubscriptionBase::SharedPtr</type>
      <name>get_subscription_by_handle</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a5f58a41f1963d12c3a5b959971bf8d28</anchor>
      <arglist>(std::shared_ptr&lt; const rcl_subscription_t &gt; subscriber_handle, const WeakNodeList &amp;weak_nodes)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static rclcpp::ServiceBase::SharedPtr</type>
      <name>get_service_by_handle</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a73131165bfd20eb378ac57f3cd26debf</anchor>
      <arglist>(std::shared_ptr&lt; const rcl_service_t &gt; service_handle, const WeakNodeList &amp;weak_nodes)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static rclcpp::ClientBase::SharedPtr</type>
      <name>get_client_by_handle</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>ae4f25ab1ec85008da9f19886daf52480</anchor>
      <arglist>(std::shared_ptr&lt; const rcl_client_t &gt; client_handle, const WeakNodeList &amp;weak_nodes)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static rclcpp::TimerBase::SharedPtr</type>
      <name>get_timer_by_handle</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a00530b95a5aabb6038d1022a069b2ce9</anchor>
      <arglist>(std::shared_ptr&lt; const rcl_timer_t &gt; timer_handle, const WeakNodeList &amp;weak_nodes)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static rclcpp::node_interfaces::NodeBaseInterface::SharedPtr</type>
      <name>get_node_by_group</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a2fcac18e77e6a8af465ccba92dc87dfd</anchor>
      <arglist>(rclcpp::CallbackGroup::SharedPtr group, const WeakNodeList &amp;weak_nodes)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static rclcpp::CallbackGroup::SharedPtr</type>
      <name>get_group_by_subscription</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>aec043e18cfd9a4dd3f07df59bd36603e</anchor>
      <arglist>(rclcpp::SubscriptionBase::SharedPtr subscription, const WeakNodeList &amp;weak_nodes)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static rclcpp::CallbackGroup::SharedPtr</type>
      <name>get_group_by_service</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>a3eaaf83d43c1763425597ddfd1589026</anchor>
      <arglist>(rclcpp::ServiceBase::SharedPtr service, const WeakNodeList &amp;weak_nodes)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static rclcpp::CallbackGroup::SharedPtr</type>
      <name>get_group_by_client</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>afb0ad889481ebfbc4eb590b7c7bc1f3c</anchor>
      <arglist>(rclcpp::ClientBase::SharedPtr client, const WeakNodeList &amp;weak_nodes)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static rclcpp::CallbackGroup::SharedPtr</type>
      <name>get_group_by_timer</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>ad396f4e2f1c99c5488f497febaf589e6</anchor>
      <arglist>(rclcpp::TimerBase::SharedPtr timer, const WeakNodeList &amp;weak_nodes)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static rclcpp::CallbackGroup::SharedPtr</type>
      <name>get_group_by_waitable</name>
      <anchorfile>classrclcpp_1_1memory__strategy_1_1MemoryStrategy.html</anchorfile>
      <anchor>ae1cc557d3f8aad5687640cf093a197a7</anchor>
      <arglist>(rclcpp::Waitable::SharedPtr waitable, const WeakNodeList &amp;weak_nodes)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::MessageInfo</name>
    <filename>classrclcpp_1_1MessageInfo.html</filename>
    <member kind="function">
      <type></type>
      <name>MessageInfo</name>
      <anchorfile>classrclcpp_1_1MessageInfo.html</anchorfile>
      <anchor>a398e49d2b3192bc562c2dd93c88bd616</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MessageInfo</name>
      <anchorfile>classrclcpp_1_1MessageInfo.html</anchorfile>
      <anchor>af0fef8ea94a8f794f4f60d8b7a94b850</anchor>
      <arglist>(const rmw_message_info_t &amp;rmw_message_info)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~MessageInfo</name>
      <anchorfile>classrclcpp_1_1MessageInfo.html</anchorfile>
      <anchor>a06caa4b205f7c5189f29230c1b83061f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rmw_message_info_t &amp;</type>
      <name>get_rmw_message_info</name>
      <anchorfile>classrclcpp_1_1MessageInfo.html</anchorfile>
      <anchor>a5b93d506866f17cd2bdb22dffcd22a1b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rmw_message_info_t &amp;</type>
      <name>get_rmw_message_info</name>
      <anchorfile>classrclcpp_1_1MessageInfo.html</anchorfile>
      <anchor>abf093c4d2075071174196e392b93dba9</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::message_memory_strategy::MessageMemoryStrategy</name>
    <filename>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; MessageT, Alloc &gt;</type>
      <name>MessageAllocTraits</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ad7bae1c755682e39f259739426529ad7</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename MessageAllocTraits::allocator_type</type>
      <name>MessageAlloc</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a80ac818d0d4ada88bb27125f179cd33a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; MessageAlloc, MessageT &gt;</type>
      <name>MessageDeleter</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa1ca9b5c95c65b249009e75aaf1e4212</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; rclcpp::SerializedMessage, Alloc &gt;</type>
      <name>SerializedMessageAllocTraits</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a412689ec1d8d82f0c130774ebf292d7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename SerializedMessageAllocTraits::allocator_type</type>
      <name>SerializedMessageAlloc</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a22fdd04ae0e4d62136ecfe55e733f558</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; SerializedMessageAlloc, rclcpp::SerializedMessage &gt;</type>
      <name>SerializedMessageDeleter</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a67091cc65776fc4dd2b5466ad16aa783</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; char, Alloc &gt;</type>
      <name>BufferAllocTraits</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a3f89c9208517ee8210aa7c4fb3f3b346</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename BufferAllocTraits::allocator_type</type>
      <name>BufferAlloc</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aec9d9a8c9eed35118096e541e2e8a69a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; BufferAlloc, char &gt;</type>
      <name>BufferDeleter</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a10f24e018b46d089b6b6d0c0da6facba</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MessageMemoryStrategy</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a87f6ffd0c5b5c192c551bbde4c5e6f73</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MessageMemoryStrategy</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa84cf1a126be2605635f280670610415</anchor>
      <arglist>(std::shared_ptr&lt; Alloc &gt; allocator)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~MessageMemoryStrategy</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>af07d65b163782d6b26449194f7ec3500</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual std::shared_ptr&lt; MessageT &gt;</type>
      <name>borrow_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ad2240bd4b2da8c31795fa322730a27ac</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual std::shared_ptr&lt; rclcpp::SerializedMessage &gt;</type>
      <name>borrow_serialized_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ab55c6b3dd7d43cd7340e5bcab17833f2</anchor>
      <arglist>(size_t capacity)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual std::shared_ptr&lt; rclcpp::SerializedMessage &gt;</type>
      <name>borrow_serialized_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>af7e42108be6b9b0d77f71ab5296f258a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>set_default_buffer_capacity</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a5823a4b6d0364e991fcae9d02abb8bea</anchor>
      <arglist>(size_t capacity)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>return_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa3ff3b67651fd522c2299e9f6a1a1357</anchor>
      <arglist>(std::shared_ptr&lt; MessageT &gt; &amp;msg)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>return_serialized_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a44d9fc29ca9b971567d1e99d334b9728</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SerializedMessage &gt; &amp;serialized_msg)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static SharedPtr</type>
      <name>create_default</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a7f17f57850f3b912ac952f99fbe2aaa7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; MessageAlloc &gt;</type>
      <name>message_allocator_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ac67991dd0bfa886b641a157ddb086661</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>MessageDeleter</type>
      <name>message_deleter_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a0520ad5e2e50fca032a8692c80ff17a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; SerializedMessageAlloc &gt;</type>
      <name>serialized_message_allocator_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa8c109457e9f5796b4e69d4803f29455</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>SerializedMessageDeleter</type>
      <name>serialized_message_deleter_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa2d6178717c39bbc07a078dee547b120</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; BufferAlloc &gt;</type>
      <name>buffer_allocator_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a384d390c2ce300aa7e2ef5974a9179ab</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>BufferDeleter</type>
      <name>buffer_deleter_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>adf7ea5cfe204bd0400c8368b54eeeee4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>size_t</type>
      <name>default_buffer_capacity_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ab4a1755b9c5bc0e42e2a5cb6a80f8992</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rcutils_allocator_t</type>
      <name>rcutils_allocator_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ac4fb2bf2b7720b8294411c7c18827da7</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>MessageMemoryStrategy&lt; CallbackMessageT, std::allocator&lt; void &gt; &gt;</name>
    <filename>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</filename>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; CallbackMessageT, std::allocator&lt; void &gt; &gt;</type>
      <name>MessageAllocTraits</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ad7bae1c755682e39f259739426529ad7</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename MessageAllocTraits::allocator_type</type>
      <name>MessageAlloc</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a80ac818d0d4ada88bb27125f179cd33a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; MessageAlloc, CallbackMessageT &gt;</type>
      <name>MessageDeleter</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa1ca9b5c95c65b249009e75aaf1e4212</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; rclcpp::SerializedMessage, std::allocator&lt; void &gt; &gt;</type>
      <name>SerializedMessageAllocTraits</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a412689ec1d8d82f0c130774ebf292d7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename SerializedMessageAllocTraits::allocator_type</type>
      <name>SerializedMessageAlloc</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a22fdd04ae0e4d62136ecfe55e733f558</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; SerializedMessageAlloc, rclcpp::SerializedMessage &gt;</type>
      <name>SerializedMessageDeleter</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a67091cc65776fc4dd2b5466ad16aa783</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; char, std::allocator&lt; void &gt; &gt;</type>
      <name>BufferAllocTraits</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a3f89c9208517ee8210aa7c4fb3f3b346</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename BufferAllocTraits::allocator_type</type>
      <name>BufferAlloc</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aec9d9a8c9eed35118096e541e2e8a69a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; BufferAlloc, char &gt;</type>
      <name>BufferDeleter</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a10f24e018b46d089b6b6d0c0da6facba</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MessageMemoryStrategy</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a87f6ffd0c5b5c192c551bbde4c5e6f73</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MessageMemoryStrategy</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa84cf1a126be2605635f280670610415</anchor>
      <arglist>(std::shared_ptr&lt; std::allocator&lt; void &gt; &gt; allocator)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~MessageMemoryStrategy</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>af07d65b163782d6b26449194f7ec3500</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual std::shared_ptr&lt; CallbackMessageT &gt;</type>
      <name>borrow_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ad2240bd4b2da8c31795fa322730a27ac</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual std::shared_ptr&lt; rclcpp::SerializedMessage &gt;</type>
      <name>borrow_serialized_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ab55c6b3dd7d43cd7340e5bcab17833f2</anchor>
      <arglist>(size_t capacity)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual std::shared_ptr&lt; rclcpp::SerializedMessage &gt;</type>
      <name>borrow_serialized_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>af7e42108be6b9b0d77f71ab5296f258a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>set_default_buffer_capacity</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a5823a4b6d0364e991fcae9d02abb8bea</anchor>
      <arglist>(size_t capacity)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>return_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa3ff3b67651fd522c2299e9f6a1a1357</anchor>
      <arglist>(std::shared_ptr&lt; CallbackMessageT &gt; &amp;msg)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>return_serialized_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a44d9fc29ca9b971567d1e99d334b9728</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SerializedMessage &gt; &amp;serialized_msg)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static SharedPtr</type>
      <name>create_default</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a7f17f57850f3b912ac952f99fbe2aaa7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; MessageAlloc &gt;</type>
      <name>message_allocator_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ac67991dd0bfa886b641a157ddb086661</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>MessageDeleter</type>
      <name>message_deleter_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a0520ad5e2e50fca032a8692c80ff17a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; SerializedMessageAlloc &gt;</type>
      <name>serialized_message_allocator_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa8c109457e9f5796b4e69d4803f29455</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>SerializedMessageDeleter</type>
      <name>serialized_message_deleter_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa2d6178717c39bbc07a078dee547b120</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; BufferAlloc &gt;</type>
      <name>buffer_allocator_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a384d390c2ce300aa7e2ef5974a9179ab</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>BufferDeleter</type>
      <name>buffer_deleter_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>adf7ea5cfe204bd0400c8368b54eeeee4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>size_t</type>
      <name>default_buffer_capacity_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ab4a1755b9c5bc0e42e2a5cb6a80f8992</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rcutils_allocator_t</type>
      <name>rcutils_allocator_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ac4fb2bf2b7720b8294411c7c18827da7</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>MessageMemoryStrategy&lt; MessageT &gt;</name>
    <filename>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</filename>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; MessageT, std::allocator&lt; void &gt; &gt;</type>
      <name>MessageAllocTraits</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ad7bae1c755682e39f259739426529ad7</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename MessageAllocTraits::allocator_type</type>
      <name>MessageAlloc</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a80ac818d0d4ada88bb27125f179cd33a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; MessageAlloc, MessageT &gt;</type>
      <name>MessageDeleter</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa1ca9b5c95c65b249009e75aaf1e4212</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; rclcpp::SerializedMessage, std::allocator&lt; void &gt; &gt;</type>
      <name>SerializedMessageAllocTraits</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a412689ec1d8d82f0c130774ebf292d7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename SerializedMessageAllocTraits::allocator_type</type>
      <name>SerializedMessageAlloc</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a22fdd04ae0e4d62136ecfe55e733f558</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; SerializedMessageAlloc, rclcpp::SerializedMessage &gt;</type>
      <name>SerializedMessageDeleter</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a67091cc65776fc4dd2b5466ad16aa783</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; char, std::allocator&lt; void &gt; &gt;</type>
      <name>BufferAllocTraits</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a3f89c9208517ee8210aa7c4fb3f3b346</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename BufferAllocTraits::allocator_type</type>
      <name>BufferAlloc</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aec9d9a8c9eed35118096e541e2e8a69a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; BufferAlloc, char &gt;</type>
      <name>BufferDeleter</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a10f24e018b46d089b6b6d0c0da6facba</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MessageMemoryStrategy</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a87f6ffd0c5b5c192c551bbde4c5e6f73</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>MessageMemoryStrategy</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa84cf1a126be2605635f280670610415</anchor>
      <arglist>(std::shared_ptr&lt; std::allocator&lt; void &gt; &gt; allocator)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~MessageMemoryStrategy</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>af07d65b163782d6b26449194f7ec3500</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual std::shared_ptr&lt; MessageT &gt;</type>
      <name>borrow_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ad2240bd4b2da8c31795fa322730a27ac</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual std::shared_ptr&lt; rclcpp::SerializedMessage &gt;</type>
      <name>borrow_serialized_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ab55c6b3dd7d43cd7340e5bcab17833f2</anchor>
      <arglist>(size_t capacity)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual std::shared_ptr&lt; rclcpp::SerializedMessage &gt;</type>
      <name>borrow_serialized_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>af7e42108be6b9b0d77f71ab5296f258a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>set_default_buffer_capacity</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a5823a4b6d0364e991fcae9d02abb8bea</anchor>
      <arglist>(size_t capacity)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>return_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa3ff3b67651fd522c2299e9f6a1a1357</anchor>
      <arglist>(std::shared_ptr&lt; MessageT &gt; &amp;msg)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>return_serialized_message</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a44d9fc29ca9b971567d1e99d334b9728</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SerializedMessage &gt; &amp;serialized_msg)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static SharedPtr</type>
      <name>create_default</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a7f17f57850f3b912ac952f99fbe2aaa7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; MessageAlloc &gt;</type>
      <name>message_allocator_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ac67991dd0bfa886b641a157ddb086661</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>MessageDeleter</type>
      <name>message_deleter_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a0520ad5e2e50fca032a8692c80ff17a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; SerializedMessageAlloc &gt;</type>
      <name>serialized_message_allocator_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa8c109457e9f5796b4e69d4803f29455</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>SerializedMessageDeleter</type>
      <name>serialized_message_deleter_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>aa2d6178717c39bbc07a078dee547b120</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; BufferAlloc &gt;</type>
      <name>buffer_allocator_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>a384d390c2ce300aa7e2ef5974a9179ab</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>BufferDeleter</type>
      <name>buffer_deleter_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>adf7ea5cfe204bd0400c8368b54eeeee4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>size_t</type>
      <name>default_buffer_capacity_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ab4a1755b9c5bc0e42e2a5cb6a80f8992</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rcutils_allocator_t</type>
      <name>rcutils_allocator_</name>
      <anchorfile>classrclcpp_1_1message__memory__strategy_1_1MessageMemoryStrategy.html</anchorfile>
      <anchor>ac4fb2bf2b7720b8294411c7c18827da7</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy</name>
    <filename>classrclcpp_1_1strategies_1_1message__pool__memory__strategy_1_1MessagePoolMemoryStrategy.html</filename>
    <templarg></templarg>
    <templarg>Size</templarg>
    <templarg></templarg>
    <base>MessageMemoryStrategy&lt; MessageT &gt;</base>
    <class kind="struct">rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy::PoolMember</class>
    <member kind="function">
      <type></type>
      <name>MessagePoolMemoryStrategy</name>
      <anchorfile>classrclcpp_1_1strategies_1_1message__pool__memory__strategy_1_1MessagePoolMemoryStrategy.html</anchorfile>
      <anchor>a8d42fc9cf4b80947d148ff50745b5e1f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; MessageT &gt;</type>
      <name>borrow_message</name>
      <anchorfile>classrclcpp_1_1strategies_1_1message__pool__memory__strategy_1_1MessagePoolMemoryStrategy.html</anchorfile>
      <anchor>aa09521e699500c448bc710b5e61bebab</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>return_message</name>
      <anchorfile>classrclcpp_1_1strategies_1_1message__pool__memory__strategy_1_1MessagePoolMemoryStrategy.html</anchorfile>
      <anchor>aecf2f56ca075a6a51d8da9e1c66bdefb</anchor>
      <arglist>(std::shared_ptr&lt; MessageT &gt; &amp;msg)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::array&lt; PoolMember, Size &gt;</type>
      <name>pool_</name>
      <anchorfile>classrclcpp_1_1strategies_1_1message__pool__memory__strategy_1_1MessagePoolMemoryStrategy.html</anchorfile>
      <anchor>ad32aa9fac1ddfed2c95f9b9c43507a0e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>size_t</type>
      <name>next_array_index_</name>
      <anchorfile>classrclcpp_1_1strategies_1_1message__pool__memory__strategy_1_1MessagePoolMemoryStrategy.html</anchorfile>
      <anchor>a34e1296a60ea253e0380e3c86c7fe8bd</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::executors::MultiThreadedExecutor</name>
    <filename>classrclcpp_1_1executors_1_1MultiThreadedExecutor.html</filename>
    <base>rclcpp::Executor</base>
    <member kind="function">
      <type></type>
      <name>MultiThreadedExecutor</name>
      <anchorfile>classrclcpp_1_1executors_1_1MultiThreadedExecutor.html</anchorfile>
      <anchor>a80be59b4434a5c16f1c0a64aa3cf06aa</anchor>
      <arglist>(const rclcpp::ExecutorOptions &amp;options=rclcpp::ExecutorOptions(), size_t number_of_threads=0, bool yield_before_execute=false, std::chrono::nanoseconds timeout=std::chrono::nanoseconds(-1))</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~MultiThreadedExecutor</name>
      <anchorfile>classrclcpp_1_1executors_1_1MultiThreadedExecutor.html</anchorfile>
      <anchor>af16dd71f081272a4fb86f9310d0e0ac0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin</name>
      <anchorfile>classrclcpp_1_1executors_1_1MultiThreadedExecutor.html</anchorfile>
      <anchor>a7f14b56a547fb58ebe9aa49fc07a83cf</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_number_of_threads</name>
      <anchorfile>classrclcpp_1_1executors_1_1MultiThreadedExecutor.html</anchorfile>
      <anchor>ac2f8cdfe2bf40901804c37545de30aa9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>run</name>
      <anchorfile>classrclcpp_1_1executors_1_1MultiThreadedExecutor.html</anchorfile>
      <anchor>aa59e0bc078ccde6b407c8c956de1eba6</anchor>
      <arglist>(size_t this_thread_number)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::detail::MutexTwoPriorities</name>
    <filename>classrclcpp_1_1detail_1_1MutexTwoPriorities.html</filename>
    <class kind="class">rclcpp::detail::MutexTwoPriorities::HighPriorityLockable</class>
    <class kind="class">rclcpp::detail::MutexTwoPriorities::LowPriorityLockable</class>
    <member kind="function">
      <type>HighPriorityLockable</type>
      <name>get_high_priority_lockable</name>
      <anchorfile>classrclcpp_1_1detail_1_1MutexTwoPriorities.html</anchorfile>
      <anchor>ac84551b3eba92b3be56864104da6ddb5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>LowPriorityLockable</type>
      <name>get_low_priority_lockable</name>
      <anchorfile>classrclcpp_1_1detail_1_1MutexTwoPriorities.html</anchorfile>
      <anchor>ae04cb82f622a16f34b526d8db5cdc49c</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::NameValidationError</name>
    <filename>classrclcpp_1_1exceptions_1_1NameValidationError.html</filename>
    <base>std::invalid_argument</base>
    <member kind="function">
      <type></type>
      <name>NameValidationError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1NameValidationError.html</anchorfile>
      <anchor>a382629ffada528023d56005510aab9ef</anchor>
      <arglist>(const char *name_type_, const char *name_, const char *error_msg_, size_t invalid_index_)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::string</type>
      <name>format_error</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1NameValidationError.html</anchorfile>
      <anchor>a6192bcecf325ba97849a24ff9ec778a2</anchor>
      <arglist>(const char *name_type, const char *name, const char *error_msg, size_t invalid_index)</arglist>
    </member>
    <member kind="variable">
      <type>const std::string</type>
      <name>name_type</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1NameValidationError.html</anchorfile>
      <anchor>a2459a2ca32ee9151f6c88d99329dc547</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const std::string</type>
      <name>name</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1NameValidationError.html</anchorfile>
      <anchor>ab1f782552bbaa1e8f4fffd0f39f917b0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const std::string</type>
      <name>error_msg</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1NameValidationError.html</anchorfile>
      <anchor>a3e67313d245dfb008c1e09cfac739a03</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const size_t</type>
      <name>invalid_index</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1NameValidationError.html</anchorfile>
      <anchor>a1476a0a673f153fde318221bdbdba0ca</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Node</name>
    <filename>classrclcpp_1_1Node.html</filename>
    <base>enable_shared_from_this&lt; Node &gt;</base>
    <member kind="typedef">
      <type>rclcpp::node_interfaces::OnSetParametersCallbackHandle</type>
      <name>OnSetParametersCallbackHandle</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>aac7e1fe50bc2683380cd9a635ac51edb</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType</type>
      <name>OnParametersSetCallbackType</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>ab403fecdb5ec2eb9f88e3eb7ae6f6d0c</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Node</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>abaa409bd2dc2cc2437fd9e47378cc5af</anchor>
      <arglist>(const std::string &amp;node_name, const NodeOptions &amp;options=NodeOptions())</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Node</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a83f5f45604217b3dc583dac7ab3b66a8</anchor>
      <arglist>(const std::string &amp;node_name, const std::string &amp;namespace_, const NodeOptions &amp;options=NodeOptions())</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Node</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a63584993927b11869d15a34af7e64937</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_name</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a1e162104495b21f50c1e9aa11460fb93</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_namespace</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a7be04e55e2934e05df992b188d9dce6d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_fully_qualified_name</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a57f81aa79665cd9d16a591bddf762747</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Logger</type>
      <name>get_logger</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a884ebededa2125cb8316998852ca3fc5</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::CallbackGroup::SharedPtr</type>
      <name>create_callback_group</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a90010521bef69072ec5008adddaf10a2</anchor>
      <arglist>(rclcpp::CallbackGroupType group_type)</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; rclcpp::CallbackGroup::WeakPtr &gt; &amp;</type>
      <name>get_callback_groups</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a90c0277b37f573e025853aea9a30b64f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; PublisherT &gt;</type>
      <name>create_publisher</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>ad1dfc9d04d67ab93353e04a7df72bc9a</anchor>
      <arglist>(const std::string &amp;topic_name, const rclcpp::QoS &amp;qos, const PublisherOptionsWithAllocator&lt; AllocatorT &gt; &amp;options=PublisherOptionsWithAllocator&lt; AllocatorT &gt;())</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; SubscriptionT &gt;</type>
      <name>create_subscription</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a82f97ad29e3d54c91f6ef3265a8636d1</anchor>
      <arglist>(const std::string &amp;topic_name, const rclcpp::QoS &amp;qos, CallbackT &amp;&amp;callback, const SubscriptionOptionsWithAllocator&lt; AllocatorT &gt; &amp;options=SubscriptionOptionsWithAllocator&lt; AllocatorT &gt;(), typename MessageMemoryStrategyT::SharedPtr msg_mem_strat=(MessageMemoryStrategyT::create_default()))</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::WallTimer&lt; CallbackT &gt;::SharedPtr</type>
      <name>create_wall_timer</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>ad27d1f6d1c5ff633b1b42ab6f77479c5</anchor>
      <arglist>(std::chrono::duration&lt; DurationRepT, DurationT &gt; period, CallbackT callback, rclcpp::CallbackGroup::SharedPtr group=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Client&lt; ServiceT &gt;::SharedPtr</type>
      <name>create_client</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>aed42f345ae1de3a1979d5a8076127199</anchor>
      <arglist>(const std::string &amp;service_name, const rmw_qos_profile_t &amp;qos_profile=rmw_qos_profile_services_default, rclcpp::CallbackGroup::SharedPtr group=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Service&lt; ServiceT &gt;::SharedPtr</type>
      <name>create_service</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a20d1f389ba5e4a57559203d5d9e77ec9</anchor>
      <arglist>(const std::string &amp;service_name, CallbackT &amp;&amp;callback, const rmw_qos_profile_t &amp;qos_profile=rmw_qos_profile_services_default, rclcpp::CallbackGroup::SharedPtr group=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>const rclcpp::ParameterValue &amp;</type>
      <name>declare_parameter</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a095ea977b26e7464d9371efea5f36c42</anchor>
      <arglist>(const std::string &amp;name, const rclcpp::ParameterValue &amp;default_value=rclcpp::ParameterValue(), const rcl_interfaces::msg::ParameterDescriptor &amp;parameter_descriptor=rcl_interfaces::msg::ParameterDescriptor(), bool ignore_override=false)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>declare_parameter</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a5b1ca8c48e28bd1f3e5fda2ab3116efd</anchor>
      <arglist>(const std::string &amp;name, const ParameterT &amp;default_value, const rcl_interfaces::msg::ParameterDescriptor &amp;parameter_descriptor=rcl_interfaces::msg::ParameterDescriptor(), bool ignore_override=false)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; ParameterT &gt;</type>
      <name>declare_parameters</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a5ebda51ed6fa4ce8f2cddc9f77e9674e</anchor>
      <arglist>(const std::string &amp;namespace_, const std::map&lt; std::string, ParameterT &gt; &amp;parameters, bool ignore_overrides=false)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; ParameterT &gt;</type>
      <name>declare_parameters</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a9b3300ad15cdf9e22ee7bfc844b9a30c</anchor>
      <arglist>(const std::string &amp;namespace_, const std::map&lt; std::string, std::pair&lt; ParameterT, rcl_interfaces::msg::ParameterDescriptor &gt; &gt; &amp;parameters, bool ignore_overrides=false)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>undeclare_parameter</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a2c525bd10070bd025bd570e98afd0d7d</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>has_parameter</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>aabb818bb5c848c5a980ddbe745517b7c</anchor>
      <arglist>(const std::string &amp;name) const</arglist>
    </member>
    <member kind="function">
      <type>rcl_interfaces::msg::SetParametersResult</type>
      <name>set_parameter</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a8c753036cd5cbdf0168ca3b39a193223</anchor>
      <arglist>(const rclcpp::Parameter &amp;parameter)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rcl_interfaces::msg::SetParametersResult &gt;</type>
      <name>set_parameters</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a7d8af4dc449c7130ccc396814b86c14d</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameters)</arglist>
    </member>
    <member kind="function">
      <type>rcl_interfaces::msg::SetParametersResult</type>
      <name>set_parameters_atomically</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a0a6d4b50c0fab975859590c82931e9ae</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameters)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Parameter</type>
      <name>get_parameter</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a0bf1077fba623d72fe1b49805f6c0a5a</anchor>
      <arglist>(const std::string &amp;name) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>get_parameter</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a1fe80d17fb6789dec1ecf74c4d135aad</anchor>
      <arglist>(const std::string &amp;name, rclcpp::Parameter &amp;parameter) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>get_parameter</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a3158e5520c58531ae4d85a2e98cdf210</anchor>
      <arglist>(const std::string &amp;name, ParameterT &amp;parameter) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>get_parameter_or</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a4bba086f9d74de0f1f10ffb8deb4c11a</anchor>
      <arglist>(const std::string &amp;name, ParameterT &amp;parameter, const ParameterT &amp;alternative_value) const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rclcpp::Parameter &gt;</type>
      <name>get_parameters</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a25890d01a2cd47ce99af887f556c529b</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;names) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>get_parameters</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>ae5ab12777100f65bd09163814dbbf486</anchor>
      <arglist>(const std::string &amp;prefix, std::map&lt; std::string, ParameterT &gt; &amp;values) const</arglist>
    </member>
    <member kind="function">
      <type>rcl_interfaces::msg::ParameterDescriptor</type>
      <name>describe_parameter</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a39de814a51b5792e86cd9b315e631fe0</anchor>
      <arglist>(const std::string &amp;name) const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rcl_interfaces::msg::ParameterDescriptor &gt;</type>
      <name>describe_parameters</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>adbfb47c0983e14482c39159b274f6308</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;names) const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; uint8_t &gt;</type>
      <name>get_parameter_types</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>af9855812e7873bd35b28afa420d4242b</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;names) const</arglist>
    </member>
    <member kind="function">
      <type>rcl_interfaces::msg::ListParametersResult</type>
      <name>list_parameters</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a37aa95886a706c174db77c2b160f6d7d</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;prefixes, uint64_t depth) const</arglist>
    </member>
    <member kind="function">
      <type>RCUTILS_WARN_UNUSED OnSetParametersCallbackHandle::SharedPtr</type>
      <name>add_on_set_parameters_callback</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a12d535bced9f26b65c0a450e6f40aff8</anchor>
      <arglist>(OnParametersSetCallbackType callback)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_on_set_parameters_callback</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a15d4f7fa0ef760941b6a78f42cccb7e0</anchor>
      <arglist>(const OnSetParametersCallbackHandle *const handler)</arglist>
    </member>
    <member kind="function">
      <type>OnParametersSetCallbackType</type>
      <name>set_on_parameters_set_callback</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a9f742edc00998b24d0a52f537bc95ad8</anchor>
      <arglist>(rclcpp::Node::OnParametersSetCallbackType callback)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>get_node_names</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a9e4302d8e28f17078cbc3b6db93c97ba</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::map&lt; std::string, std::vector&lt; std::string &gt; &gt;</type>
      <name>get_topic_names_and_types</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>ab64ee81d61071100f222b819eab3e311</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::map&lt; std::string, std::vector&lt; std::string &gt; &gt;</type>
      <name>get_service_names_and_types</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a0d400bbd606a936a556b747f519eab38</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::map&lt; std::string, std::vector&lt; std::string &gt; &gt;</type>
      <name>get_service_names_and_types_by_node</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>aa3df8a7d8b174f8049dfd2766f4b2eb9</anchor>
      <arglist>(const std::string &amp;node_name, const std::string &amp;namespace_) const</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>count_publishers</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a8792429fa0ee5c5d7f9eca537cf4cff0</anchor>
      <arglist>(const std::string &amp;topic_name) const</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>count_subscribers</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a7aa2ed22450d2f3300ea5b879acd73b8</anchor>
      <arglist>(const std::string &amp;topic_name) const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rclcpp::TopicEndpointInfo &gt;</type>
      <name>get_publishers_info_by_topic</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a08220daf1d66596012e858d19efd1212</anchor>
      <arglist>(const std::string &amp;topic_name, bool no_mangle=false) const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rclcpp::TopicEndpointInfo &gt;</type>
      <name>get_subscriptions_info_by_topic</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>afba993c401901433b3c8647ef716c7e4</anchor>
      <arglist>(const std::string &amp;topic_name, bool no_mangle=false) const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Event::SharedPtr</type>
      <name>get_graph_event</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a9e5851844ecb1794503b8fc0f9f5c898</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>wait_for_graph_change</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>ab72e2c59951c08c515fd597bfc9e8cfd</anchor>
      <arglist>(rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Clock::SharedPtr</type>
      <name>get_clock</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>aec0332887ae90caed727e5025cb62195</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Clock::ConstSharedPtr</type>
      <name>get_clock</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a2c725f0bb2e36943ca5c0255ad0bc0cf</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Time</type>
      <name>now</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a1db9cec1af19a994b3f1dad2059d2707</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::node_interfaces::NodeBaseInterface::SharedPtr</type>
      <name>get_node_base_interface</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a37e5b136ef0a29853e01d5cecb8e4690</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::node_interfaces::NodeClockInterface::SharedPtr</type>
      <name>get_node_clock_interface</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a5495b9faae261feed33d8d8828840709</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::node_interfaces::NodeGraphInterface::SharedPtr</type>
      <name>get_node_graph_interface</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a2378e5b1486e58afdead52fa14928c2b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr</type>
      <name>get_node_logging_interface</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>af05394f5147dd263af2df15a99f5a4bb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::node_interfaces::NodeTimersInterface::SharedPtr</type>
      <name>get_node_timers_interface</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>aa28869f473b562a08aa581fc7deae867</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr</type>
      <name>get_node_topics_interface</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a5ae76be5bac60aea82486a9b4c6b2ff5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::node_interfaces::NodeServicesInterface::SharedPtr</type>
      <name>get_node_services_interface</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a588cb7990fd658e58c532d65e4872df3</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr</type>
      <name>get_node_waitables_interface</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a6708bb439e1d3bb1ea44fbae50e460b2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::node_interfaces::NodeParametersInterface::SharedPtr</type>
      <name>get_node_parameters_interface</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>ab6272f2facdfb35f0ef9ce01ffedeec9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr</type>
      <name>get_node_time_source_interface</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>afa70b6e8bd3ecc39c5928d822024db39</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>get_sub_namespace</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>aa9ac5055f3f7a3537642482bd505d4bd</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>get_effective_namespace</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>afe4790bba38942b795d24b6ccb3e9c5a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Node::SharedPtr</type>
      <name>create_sub_node</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a1df8e4507d17a503630317432449d69d</anchor>
      <arglist>(const std::string &amp;sub_namespace)</arglist>
    </member>
    <member kind="function">
      <type>const rclcpp::NodeOptions &amp;</type>
      <name>get_node_options</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a53b6a37589d4371c9e0675acfc0f197a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Client&lt; ServiceT &gt;::SharedPtr</type>
      <name>create_client</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>ac9aed5a03eb2420254745528a7d20443</anchor>
      <arglist>(const std::string &amp;service_name, const rmw_qos_profile_t &amp;qos_profile, rclcpp::CallbackGroup::SharedPtr group)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>Node</name>
      <anchorfile>classrclcpp_1_1Node.html</anchorfile>
      <anchor>a52e594aa6a2790c0190739d862f68270</anchor>
      <arglist>(const Node &amp;other, const std::string &amp;sub_namespace)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::graph_listener::NodeAlreadyAddedError</name>
    <filename>classrclcpp_1_1graph__listener_1_1NodeAlreadyAddedError.html</filename>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>NodeAlreadyAddedError</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1NodeAlreadyAddedError.html</anchorfile>
      <anchor>a7bf67c2b9b8d5c08228af2b11ad3eabc</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeBase</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeBase.html</filename>
    <base>rclcpp::node_interfaces::NodeBaseInterface</base>
    <member kind="function">
      <type></type>
      <name>NodeBase</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>ab225350134f5186df4c118ca60262e2f</anchor>
      <arglist>(const std::string &amp;node_name, const std::string &amp;namespace_, rclcpp::Context::SharedPtr context, const rcl_node_options_t &amp;rcl_node_options, bool use_intra_process_default, bool enable_topic_statistics_default)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeBase</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>aee81442e1ddb608bbd18861ecfb2f4e6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_name</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>a1eda3559197cebda3939380478664cd8</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_namespace</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>a9d7b7e722b084bb75e23e68f68662086</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_fully_qualified_name</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>a8469a2ea72cbbafdaf4474d94845062d</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Context::SharedPtr</type>
      <name>get_context</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>a64a83ab7321b555b8d140d7bb903d36c</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>rcl_node_t *</type>
      <name>get_rcl_node_handle</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>ab32b9a2214fae00ad08a0478133f3232</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>const rcl_node_t *</type>
      <name>get_rcl_node_handle</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>a65d50dbb2bbecdaa7dbef9155adc3ee7</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rcl_node_t &gt;</type>
      <name>get_shared_rcl_node_handle</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>ab82d9879fe7b9886ededafad555bfc7e</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; const rcl_node_t &gt;</type>
      <name>get_shared_rcl_node_handle</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>a5e99c2692141cfa648afabd1c35a80aa</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::CallbackGroup::SharedPtr</type>
      <name>create_callback_group</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>a38276574ee82366ce693c3a573eb6dd5</anchor>
      <arglist>(rclcpp::CallbackGroupType group_type) override</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::CallbackGroup::SharedPtr</type>
      <name>get_default_callback_group</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>ab817e92a714e1612150b465708925d14</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>callback_group_in_node</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>ac777d661b73889084d7fadd3b70d838b</anchor>
      <arglist>(rclcpp::CallbackGroup::SharedPtr group) override</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; rclcpp::CallbackGroup::WeakPtr &gt; &amp;</type>
      <name>get_callback_groups</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>a9d10408894f61cb3694d086fcec38521</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>std::atomic_bool &amp;</type>
      <name>get_associated_with_executor_atomic</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>a7527a7a472c3090b54142980dd3d5661</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>rcl_guard_condition_t *</type>
      <name>get_notify_guard_condition</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>a4d95fc3c87aa8479e0e113ad85f37b1f</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::unique_lock&lt; std::recursive_mutex &gt;</type>
      <name>acquire_notify_guard_condition_lock</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>abd316ced65b34b949828b6abec16ad2e</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>get_use_intra_process_default</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>ab43b1ea468fb67b0d1762acfeb696dc7</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>get_enable_topic_statistics_default</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBase.html</anchorfile>
      <anchor>aa66e376d24fbf3c9dc27aa29817c6a66</anchor>
      <arglist>() const override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeBaseInterface</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeBaseInterface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>aa16a868368cfa38bb5c459ff45b04282</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual const char *</type>
      <name>get_name</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>adb95f8c640125c6251e5848375a751ad</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual const char *</type>
      <name>get_namespace</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>a053bbf537c9a8b75458c29deb8139d4e</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual const char *</type>
      <name>get_fully_qualified_name</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>afa4d5fcdca7d9533765f31d121b24767</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rclcpp::Context::SharedPtr</type>
      <name>get_context</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>a3c2e708704b9c953d684dabe6e250e54</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rcl_node_t *</type>
      <name>get_rcl_node_handle</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>af379fa01a7e2c32d12572434680a7bfc</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual const rcl_node_t *</type>
      <name>get_rcl_node_handle</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>a179644d42c3502df5120948ef26336fa</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::shared_ptr&lt; rcl_node_t &gt;</type>
      <name>get_shared_rcl_node_handle</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>abe5afb88f0bca7fac881aa3b06c82029</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::shared_ptr&lt; const rcl_node_t &gt;</type>
      <name>get_shared_rcl_node_handle</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>addf7daabd9758fe9251fa798e221ecae</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rclcpp::CallbackGroup::SharedPtr</type>
      <name>create_callback_group</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>a99e2d48c741cc6b3c7c03e7ca178009c</anchor>
      <arglist>(rclcpp::CallbackGroupType group_type)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rclcpp::CallbackGroup::SharedPtr</type>
      <name>get_default_callback_group</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>a4b6445f0df43066db75d2941ba2fddd9</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>callback_group_in_node</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>a2d0958511a78abdada5c18e4016b3727</anchor>
      <arglist>(rclcpp::CallbackGroup::SharedPtr group)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual const std::vector&lt; rclcpp::CallbackGroup::WeakPtr &gt; &amp;</type>
      <name>get_callback_groups</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>a0e94f2cc05b448bde81d5a9654675698</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::atomic_bool &amp;</type>
      <name>get_associated_with_executor_atomic</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>a78a5f408f41d92924a8435cc254dc15a</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rcl_guard_condition_t *</type>
      <name>get_notify_guard_condition</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>aaea735e8d7a7a3cbea91522ae6f2bf97</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::unique_lock&lt; std::recursive_mutex &gt;</type>
      <name>acquire_notify_guard_condition_lock</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>ab2eb5fc9afb1111b6489181b1bb7ed76</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>get_use_intra_process_default</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>adb43088e1fa2f637959edc6d474fb32f</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>get_enable_topic_statistics_default</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeBaseInterface.html</anchorfile>
      <anchor>a9bf09b9fe62763e5abec378d6e8b3b3d</anchor>
      <arglist>() const =0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeClock</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeClock.html</filename>
    <base>rclcpp::node_interfaces::NodeClockInterface</base>
    <member kind="function">
      <type></type>
      <name>NodeClock</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeClock.html</anchorfile>
      <anchor>ab217a7086ed64a0161a5cd6fae4dadbe</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base, rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics, rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph, rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services, rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeClock</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeClock.html</anchorfile>
      <anchor>a748aeb36ab0eb6b3084c3338ceae78a7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Clock::SharedPtr</type>
      <name>get_clock</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeClock.html</anchorfile>
      <anchor>a0998d27ad1fceaa0af4f101f54177f1a</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Clock::ConstSharedPtr</type>
      <name>get_clock</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeClock.html</anchorfile>
      <anchor>a837985851915a766bf9b036f52451736</anchor>
      <arglist>() const override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeClockInterface</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeClockInterface.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeClockInterface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeClockInterface.html</anchorfile>
      <anchor>afefc6234ee991f31f6606d0b8bf3faef</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rclcpp::Clock::SharedPtr</type>
      <name>get_clock</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeClockInterface.html</anchorfile>
      <anchor>a6e4c104d8ad8bc5f9cd94a7bda28926c</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rclcpp::Clock::ConstSharedPtr</type>
      <name>get_clock</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeClockInterface.html</anchorfile>
      <anchor>aad29532e93b9de2e5e1c86f3c9c332c1</anchor>
      <arglist>() const =0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeGraph</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</filename>
    <base>rclcpp::node_interfaces::NodeGraphInterface</base>
    <member kind="function">
      <type></type>
      <name>NodeGraph</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>abea4083c438b551d7c366c26921da996</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeGraph</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>ad1ecc3b6e983cea9a9bb3b3296b10ae7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::map&lt; std::string, std::vector&lt; std::string &gt; &gt;</type>
      <name>get_topic_names_and_types</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>adf450c64125a78006d1f0d6a4da1366a</anchor>
      <arglist>(bool no_demangle=false) const override</arglist>
    </member>
    <member kind="function">
      <type>std::map&lt; std::string, std::vector&lt; std::string &gt; &gt;</type>
      <name>get_service_names_and_types</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>aa2bec35db8658c83f4b8d4662fc9e44c</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>std::map&lt; std::string, std::vector&lt; std::string &gt; &gt;</type>
      <name>get_service_names_and_types_by_node</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>a5f80f5fc9ef461c882a7e5f32c4a06eb</anchor>
      <arglist>(const std::string &amp;node_name, const std::string &amp;namespace_) const override</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>get_node_names</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>a2c21b9c5e4b53a54c84e22940d66d77b</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::pair&lt; std::string, std::string &gt; &gt;</type>
      <name>get_node_names_and_namespaces</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>a8f4e4f4241d7487f9cf230399a097954</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>count_publishers</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>ad1cad430f6099063f4149e36e338400c</anchor>
      <arglist>(const std::string &amp;topic_name) const override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>count_subscribers</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>aceb2d765d4681532f1909b9410be1452</anchor>
      <arglist>(const std::string &amp;topic_name) const override</arglist>
    </member>
    <member kind="function">
      <type>const rcl_guard_condition_t *</type>
      <name>get_graph_guard_condition</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>a8a92aaf6aff34687a431dafa4520e290</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>notify_graph_change</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>ab3fb4a7f5c875f7dd126f2ceed8b2c4c</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>notify_shutdown</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>afde53f8adde6584712705b85c09c955a</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Event::SharedPtr</type>
      <name>get_graph_event</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>a5d7406637a5de005f8d25f1b68eb29db</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>wait_for_graph_change</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>a30d5cf2d4d38cf410d72bf3ae459d6e1</anchor>
      <arglist>(rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout) override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>count_graph_users</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>a2f9d9b425facaff5c1396178458e25ae</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rclcpp::TopicEndpointInfo &gt;</type>
      <name>get_publishers_info_by_topic</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>ae191391ef7f70941f4c52256ac4e8e14</anchor>
      <arglist>(const std::string &amp;topic_name, bool no_mangle=false) const override</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rclcpp::TopicEndpointInfo &gt;</type>
      <name>get_subscriptions_info_by_topic</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraph.html</anchorfile>
      <anchor>ae7b268ba5135700db7e99bf9b7f1355b</anchor>
      <arglist>(const std::string &amp;topic_name, bool no_mangle=false) const override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeGraphInterface</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeGraphInterface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>acd12eb8eaa948669193d4a6516326527</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::map&lt; std::string, std::vector&lt; std::string &gt; &gt;</type>
      <name>get_topic_names_and_types</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>a778c5797486f5126edccd70dfe583acc</anchor>
      <arglist>(bool no_demangle=false) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::map&lt; std::string, std::vector&lt; std::string &gt; &gt;</type>
      <name>get_service_names_and_types</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>a5852d4535a2e61387ca2be874031bdc9</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::map&lt; std::string, std::vector&lt; std::string &gt; &gt;</type>
      <name>get_service_names_and_types_by_node</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>adab93c48a10d8377316d73d861310787</anchor>
      <arglist>(const std::string &amp;node_name, const std::string &amp;namespace_) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::vector&lt; std::string &gt;</type>
      <name>get_node_names</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>a3227cff0d0c306c3a7ba8550ec36c92d</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::vector&lt; std::pair&lt; std::string, std::string &gt; &gt;</type>
      <name>get_node_names_and_namespaces</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>acaebd5bddcc4f018e5250335e95bb2a0</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual size_t</type>
      <name>count_publishers</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>a916b41e6d63cf3abd929470b5f3e10b7</anchor>
      <arglist>(const std::string &amp;topic_name) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual size_t</type>
      <name>count_subscribers</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>a45e3da58dd22b42abda51d45d5e17e7c</anchor>
      <arglist>(const std::string &amp;topic_name) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual const rcl_guard_condition_t *</type>
      <name>get_graph_guard_condition</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>ac4bc15b80573f2139e131db5c66e35e0</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>notify_graph_change</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>a5bd573aeebdbc9d70ed8f0e0fc7f1049</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>notify_shutdown</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>ad4ea2c5addd1a4cd8b3866bfc935a681</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rclcpp::Event::SharedPtr</type>
      <name>get_graph_event</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>a6d80bdf76f79254fa80f1b1b527d1446</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>wait_for_graph_change</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>a6da052e3f08251b5083b6af953c71a37</anchor>
      <arglist>(rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual size_t</type>
      <name>count_graph_users</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>a91d3031a12921e4e3eecce5a106d8abf</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::vector&lt; rclcpp::TopicEndpointInfo &gt;</type>
      <name>get_publishers_info_by_topic</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>ad2b5d4374ee406329dcc431a41bfa135</anchor>
      <arglist>(const std::string &amp;topic_name, bool no_mangle=false) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::vector&lt; rclcpp::TopicEndpointInfo &gt;</type>
      <name>get_subscriptions_info_by_topic</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeGraphInterface.html</anchorfile>
      <anchor>ab81e3c07e21c1abdead0a5e0822f77e1</anchor>
      <arglist>(const std::string &amp;topic_name, bool no_mangle=false) const =0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeLogging</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeLogging.html</filename>
    <base>rclcpp::node_interfaces::NodeLoggingInterface</base>
    <member kind="function">
      <type></type>
      <name>NodeLogging</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeLogging.html</anchorfile>
      <anchor>ade3a650fd202b1f73fa3197aab85bca4</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeLogging</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeLogging.html</anchorfile>
      <anchor>a05d59e06f3f8d7687475e39991fa3618</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Logger</type>
      <name>get_logger</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeLogging.html</anchorfile>
      <anchor>a1cd290091f622e2f8dd67d419912d65c</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_logger_name</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeLogging.html</anchorfile>
      <anchor>a9923fdd1f65318f67b3ed76997d4bab2</anchor>
      <arglist>() const override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeLoggingInterface</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeLoggingInterface.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeLoggingInterface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeLoggingInterface.html</anchorfile>
      <anchor>a10ed0105cb8fefc4f69b955ee672b75c</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rclcpp::Logger</type>
      <name>get_logger</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeLoggingInterface.html</anchorfile>
      <anchor>a25422d8ba04bc7ee36ca9f3d7513bbf0</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual const char *</type>
      <name>get_logger_name</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeLoggingInterface.html</anchorfile>
      <anchor>ae7a5563ca0ca7163f03f4b821b046842</anchor>
      <arglist>() const =0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::graph_listener::NodeNotFoundError</name>
    <filename>classrclcpp_1_1graph__listener_1_1NodeNotFoundError.html</filename>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>NodeNotFoundError</name>
      <anchorfile>classrclcpp_1_1graph__listener_1_1NodeNotFoundError.html</anchorfile>
      <anchor>af52fe2cd8407512b044653643b9a9509</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::NodeOptions</name>
    <filename>classrclcpp_1_1NodeOptions.html</filename>
    <member kind="function">
      <type></type>
      <name>NodeOptions</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a7b4f05834d1cc16f8b3ef711d493c81c</anchor>
      <arglist>(rcl_allocator_t allocator=rcl_get_default_allocator())</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeOptions</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>ab4b215e7262ad31edf18cfb49250f349</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>NodeOptions</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a66e0e11de2b4325eb4dd8ab70ffd5e39</anchor>
      <arglist>(const NodeOptions &amp;other)</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>operator=</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a2df8aafa574d59f375de34184bc9ac49</anchor>
      <arglist>(const NodeOptions &amp;other)</arglist>
    </member>
    <member kind="function">
      <type>const rcl_node_options_t *</type>
      <name>get_rcl_node_options</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>afb3a8ec1c82abc57587c1e7e684d510d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Context::SharedPtr</type>
      <name>context</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>ad38d7afddbb7e78d26ddc1d20715c771</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>context</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>aa7491d3f2d3b960bbe42dd7045879a38</anchor>
      <arglist>(rclcpp::Context::SharedPtr context)</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; std::string &gt; &amp;</type>
      <name>arguments</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a925c259a2acc596424261ed0d62c298f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>arguments</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>ab0c61163fb466ceb980a001bdce2fb7a</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;arguments)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rclcpp::Parameter &gt; &amp;</type>
      <name>parameter_overrides</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a9bf33cbc399a7c99e167c10ee9aa141d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; rclcpp::Parameter &gt; &amp;</type>
      <name>parameter_overrides</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a2af06ae072c53ec81a06dfdd5771eb53</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>parameter_overrides</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a9d9de6100ebbe1e36cc6339d05b39d8c</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameter_overrides)</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>append_parameter_override</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a4f1aac507fdea0ce77859539f8270615</anchor>
      <arglist>(const std::string &amp;name, const ParameterT &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>use_global_arguments</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a2beacdf55dd3da5f0fed8d4c22cf6dfb</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>use_global_arguments</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a716a84c68dc2307b03171cc1a8663549</anchor>
      <arglist>(bool use_global_arguments)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>enable_rosout</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a5109d7a73b2e15ea81506ead56ec2c00</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>enable_rosout</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>ae4e0e160c99282dac91318fec3cc22ea</anchor>
      <arglist>(bool enable_rosout)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>use_intra_process_comms</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>aac9693056075d57cdfed0f03f990520f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>use_intra_process_comms</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>adcad44e6fef3a5d1dcc8e3bc9ccf9e2a</anchor>
      <arglist>(bool use_intra_process_comms)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>enable_topic_statistics</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>ad3ae1ef6a4c4c6ee28d520639903480e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>enable_topic_statistics</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>aceff2f380caffa4aae302897d9a3e5f4</anchor>
      <arglist>(bool enable_topic_statistics)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>start_parameter_services</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a96fe8463cc40169e25bd6a7a0a1b493a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>start_parameter_services</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a56f3800827cb0e4f136a57a2b2ba045d</anchor>
      <arglist>(bool start_parameter_services)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>start_parameter_event_publisher</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a018247a67e09417b0acd33e6d436b517</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>start_parameter_event_publisher</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a53af3d02712d89bb4f71cca53f87410f</anchor>
      <arglist>(bool start_parameter_event_publisher)</arglist>
    </member>
    <member kind="function">
      <type>const rclcpp::QoS &amp;</type>
      <name>parameter_event_qos</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>ad51d81afd7973c9b5c8fc6ace07078f9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>parameter_event_qos</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a9597dbb7387920a035919c0232c338f3</anchor>
      <arglist>(const rclcpp::QoS &amp;parameter_event_qos)</arglist>
    </member>
    <member kind="function">
      <type>const rclcpp::PublisherOptionsBase &amp;</type>
      <name>parameter_event_publisher_options</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>ab20274ddffde55876f338f1ca86c9349</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>parameter_event_publisher_options</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a133b4dcd5c6505d8dddb83eed7e2991c</anchor>
      <arglist>(const rclcpp::PublisherOptionsBase &amp;parameter_event_publisher_options)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>allow_undeclared_parameters</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a5b9580203b3cb88c75b5b858e26b3b7d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>allow_undeclared_parameters</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a203b8059759eaa4fb498abc58b1cb33b</anchor>
      <arglist>(bool allow_undeclared_parameters)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>automatically_declare_parameters_from_overrides</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a63058f5fbe1c00c62e4b16bec4028f6b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>automatically_declare_parameters_from_overrides</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a094ceb7af7c9b358ec007a4b8e14d40d</anchor>
      <arglist>(bool automatically_declare_parameters_from_overrides)</arglist>
    </member>
    <member kind="function">
      <type>const rcl_allocator_t &amp;</type>
      <name>allocator</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a79d05fd04df7cad79448a07681df96d3</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>NodeOptions &amp;</type>
      <name>allocator</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a371bb7f27e4e07933f06f4a9e0168e49</anchor>
      <arglist>(rcl_allocator_t allocator)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>size_t</type>
      <name>get_domain_id_from_env</name>
      <anchorfile>classrclcpp_1_1NodeOptions.html</anchorfile>
      <anchor>a47bd6a2f2fadbb6deba155b6f3052491</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeParameters</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</filename>
    <base>rclcpp::node_interfaces::NodeParametersInterface</base>
    <member kind="typedef">
      <type>std::list&lt; OnSetParametersCallbackHandle::WeakPtr &gt;</type>
      <name>CallbacksContainerType</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>ad43a07db49a6c7fc33884f63af82007a</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>NodeParameters</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>a9c3cd33c84df2662da4e89af02d4e380</anchor>
      <arglist>(const node_interfaces::NodeBaseInterface::SharedPtr node_base, const node_interfaces::NodeLoggingInterface::SharedPtr node_logging, const node_interfaces::NodeTopicsInterface::SharedPtr node_topics, const node_interfaces::NodeServicesInterface::SharedPtr node_services, const node_interfaces::NodeClockInterface::SharedPtr node_clock, const std::vector&lt; Parameter &gt; &amp;parameter_overrides, bool start_parameter_services, bool start_parameter_event_publisher, const rclcpp::QoS &amp;parameter_event_qos, const rclcpp::PublisherOptionsBase &amp;parameter_event_publisher_options, bool allow_undeclared_parameters, bool automatically_declare_parameters_from_overrides)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeParameters</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>a1894e748b3944a3aedde8dc3395809b6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rclcpp::ParameterValue &amp;</type>
      <name>declare_parameter</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>a3592d78f0c3c5fd87d68af2747038218</anchor>
      <arglist>(const std::string &amp;name, const rclcpp::ParameterValue &amp;default_value, const rcl_interfaces::msg::ParameterDescriptor &amp;parameter_descriptor, bool ignore_override) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>undeclare_parameter</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>a2ba79b5988501a5446a025e7c8c6e8e1</anchor>
      <arglist>(const std::string &amp;name) override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>has_parameter</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>a41ff259e3af1b72afa6956c08a7ec8d6</anchor>
      <arglist>(const std::string &amp;name) const override</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rcl_interfaces::msg::SetParametersResult &gt;</type>
      <name>set_parameters</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>ab08c71d8ad414cce5b57202dfd1f128c</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameters) override</arglist>
    </member>
    <member kind="function">
      <type>rcl_interfaces::msg::SetParametersResult</type>
      <name>set_parameters_atomically</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>a25ee9e6d7437b709fb3e31c6ac901229</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameters) override</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rclcpp::Parameter &gt;</type>
      <name>get_parameters</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>a52f2cbeb6a875395143c09bc71eac31c</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;names) const override</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Parameter</type>
      <name>get_parameter</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>a4c13355bbdbf3bd8b7e04dcddb81178c</anchor>
      <arglist>(const std::string &amp;name) const override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>get_parameter</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>a69fa54bb1a598ab086405bfa8f9f05b5</anchor>
      <arglist>(const std::string &amp;name, rclcpp::Parameter &amp;parameter) const override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>get_parameters_by_prefix</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>ad53a56f48cd5c857e2fe5eaa2978a422</anchor>
      <arglist>(const std::string &amp;prefix, std::map&lt; std::string, rclcpp::Parameter &gt; &amp;parameters) const override</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rcl_interfaces::msg::ParameterDescriptor &gt;</type>
      <name>describe_parameters</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>ae970a27f63bfe191222d6ee61fa62482</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;names) const override</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; uint8_t &gt;</type>
      <name>get_parameter_types</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>a3aaf7e7325aec9501efce9a688cbe983</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;names) const override</arglist>
    </member>
    <member kind="function">
      <type>rcl_interfaces::msg::ListParametersResult</type>
      <name>list_parameters</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>aa4412cc00c629c20394ed7f07301462a</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;prefixes, uint64_t depth) const override</arglist>
    </member>
    <member kind="function">
      <type>RCUTILS_WARN_UNUSED OnSetParametersCallbackHandle::SharedPtr</type>
      <name>add_on_set_parameters_callback</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>a1c9b9eb47402746ac09c52032e4644a4</anchor>
      <arglist>(OnParametersSetCallbackType callback) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_on_set_parameters_callback</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>ae458c040af21dcb57055b89455b52c6e</anchor>
      <arglist>(const OnSetParametersCallbackHandle *const handler) override</arglist>
    </member>
    <member kind="function">
      <type>OnParametersSetCallbackType</type>
      <name>set_on_parameters_set_callback</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>a1937592341036268cf1e7deb2abc5566</anchor>
      <arglist>(OnParametersSetCallbackType callback) override</arglist>
    </member>
    <member kind="function">
      <type>const std::map&lt; std::string, rclcpp::ParameterValue &gt; &amp;</type>
      <name>get_parameter_overrides</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParameters.html</anchorfile>
      <anchor>af0e0e4f61ac9a157de3b52eb03274aad</anchor>
      <arglist>() const override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeParametersInterface</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</filename>
    <member kind="typedef">
      <type>OnSetParametersCallbackHandle::OnParametersSetCallbackType</type>
      <name>OnParametersSetCallbackType</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a7143766bc4249673edb6244820fc5a94</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeParametersInterface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a21c336479d8fdfc746b11ce3eb58cc48</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual const rclcpp::ParameterValue &amp;</type>
      <name>declare_parameter</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a4e36900c6f6bf9bb4f42cbc491014c64</anchor>
      <arglist>(const std::string &amp;name, const rclcpp::ParameterValue &amp;default_value=rclcpp::ParameterValue(), const rcl_interfaces::msg::ParameterDescriptor &amp;parameter_descriptor=rcl_interfaces::msg::ParameterDescriptor(), bool ignore_override=false)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>undeclare_parameter</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a62729f0f5500a8b7dddc05e60641c601</anchor>
      <arglist>(const std::string &amp;name)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>has_parameter</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a6bc191818613d31557f478941aa5fd2d</anchor>
      <arglist>(const std::string &amp;name) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::vector&lt; rcl_interfaces::msg::SetParametersResult &gt;</type>
      <name>set_parameters</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a8f48428d39080e751f115b0d2262ad3f</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameters)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rcl_interfaces::msg::SetParametersResult</type>
      <name>set_parameters_atomically</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>ab0db9ee199619cbc3e6567ea15e3736b</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameters)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::vector&lt; rclcpp::Parameter &gt;</type>
      <name>get_parameters</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a8b9aba57037dbf31c1a7fd37cd43ae61</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;names) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rclcpp::Parameter</type>
      <name>get_parameter</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a185ac1c9b3946d5bdac8250ef404090a</anchor>
      <arglist>(const std::string &amp;name) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>get_parameter</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>af0ba0c92858f328706c478246c9c5153</anchor>
      <arglist>(const std::string &amp;name, rclcpp::Parameter &amp;parameter) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>get_parameters_by_prefix</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>aa95a7ac9c0d9a6eeb146a563adc0e843</anchor>
      <arglist>(const std::string &amp;prefix, std::map&lt; std::string, rclcpp::Parameter &gt; &amp;parameters) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::vector&lt; rcl_interfaces::msg::ParameterDescriptor &gt;</type>
      <name>describe_parameters</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a9223831ba746d4a87df11c9deece789a</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;names) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::vector&lt; uint8_t &gt;</type>
      <name>get_parameter_types</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a067c45427f11916f0f518afce9f21103</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;names) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rcl_interfaces::msg::ListParametersResult</type>
      <name>list_parameters</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>aea2d1f4fbf0160523c6985fd93251140</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;prefixes, uint64_t depth) const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual OnSetParametersCallbackHandle::SharedPtr</type>
      <name>add_on_set_parameters_callback</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a271ee80ae8adb9cd16622bc01426b05f</anchor>
      <arglist>(OnParametersSetCallbackType callback)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>remove_on_set_parameters_callback</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a05f6ea387228859950a12735e4d53443</anchor>
      <arglist>(const OnSetParametersCallbackHandle *const handler)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual OnParametersSetCallbackType</type>
      <name>set_on_parameters_set_callback</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a2139a5f419e35932f7cc2653e8178b59</anchor>
      <arglist>(OnParametersSetCallbackType callback)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual const std::map&lt; std::string, rclcpp::ParameterValue &gt; &amp;</type>
      <name>get_parameter_overrides</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeParametersInterface.html</anchorfile>
      <anchor>a448fec77ccff87065067ea75b1e2c87c</anchor>
      <arglist>() const =0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeServices</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeServices.html</filename>
    <base>rclcpp::node_interfaces::NodeServicesInterface</base>
    <member kind="function">
      <type></type>
      <name>NodeServices</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeServices.html</anchorfile>
      <anchor>a7f094c25de9a88cd73958ec57f137f88</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeServices</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeServices.html</anchorfile>
      <anchor>a205b980ec6cc51cde49ddce0c1c6de19</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_client</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeServices.html</anchorfile>
      <anchor>a2ded44d50eee8258637fe3000ca767d5</anchor>
      <arglist>(rclcpp::ClientBase::SharedPtr client_base_ptr, rclcpp::CallbackGroup::SharedPtr group) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_service</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeServices.html</anchorfile>
      <anchor>abdf4c7379cd1c6a51a4a8d340089a5b2</anchor>
      <arglist>(rclcpp::ServiceBase::SharedPtr service_base_ptr, rclcpp::CallbackGroup::SharedPtr group) override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeServicesInterface</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeServicesInterface.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeServicesInterface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeServicesInterface.html</anchorfile>
      <anchor>aa2e640d45de144c1f49549c029fb04f3</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>add_client</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeServicesInterface.html</anchorfile>
      <anchor>a4c658aa5c1ea43d2fa07d38a998071ca</anchor>
      <arglist>(rclcpp::ClientBase::SharedPtr client_base_ptr, rclcpp::CallbackGroup::SharedPtr group)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>add_service</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeServicesInterface.html</anchorfile>
      <anchor>ac133d59cd3b64ef9d08acbf8119f473a</anchor>
      <arglist>(rclcpp::ServiceBase::SharedPtr service_base_ptr, rclcpp::CallbackGroup::SharedPtr group)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeTimers</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeTimers.html</filename>
    <base>rclcpp::node_interfaces::NodeTimersInterface</base>
    <member kind="function">
      <type></type>
      <name>NodeTimers</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTimers.html</anchorfile>
      <anchor>a704495d02066972b5cc9ab9e02de21a7</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeTimers</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTimers.html</anchorfile>
      <anchor>a321950a107ded2c51c5e910071489a54</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_timer</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTimers.html</anchorfile>
      <anchor>a5d363c503124706df446a6661b52618e</anchor>
      <arglist>(rclcpp::TimerBase::SharedPtr timer, rclcpp::CallbackGroup::SharedPtr callback_group) override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeTimersInterface</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeTimersInterface.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeTimersInterface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTimersInterface.html</anchorfile>
      <anchor>ae119a8f871b4a9c01a3bdf611fa818a0</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>add_timer</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTimersInterface.html</anchorfile>
      <anchor>a7d2b5950f2ad8789768f6b8797e632c1</anchor>
      <arglist>(rclcpp::TimerBase::SharedPtr timer, rclcpp::CallbackGroup::SharedPtr callback_group)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeTimeSource</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeTimeSource.html</filename>
    <base>rclcpp::node_interfaces::NodeTimeSourceInterface</base>
    <member kind="function">
      <type></type>
      <name>NodeTimeSource</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTimeSource.html</anchorfile>
      <anchor>a75a2378a2ce6449bc08681c9b31e634e</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base, rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics, rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph, rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services, rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging, rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock, rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeTimeSource</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTimeSource.html</anchorfile>
      <anchor>a9b7dc20b15c9859d713ad37f96e36e8a</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeTimeSourceInterface</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeTimeSourceInterface.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeTimeSourceInterface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTimeSourceInterface.html</anchorfile>
      <anchor>a0ea3f99f78372c133acfb598dc29d09c</anchor>
      <arglist>()=default</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeTopics</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeTopics.html</filename>
    <base>rclcpp::node_interfaces::NodeTopicsInterface</base>
    <member kind="function">
      <type></type>
      <name>NodeTopics</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopics.html</anchorfile>
      <anchor>ae3c1713673580ab3e920c553b9d4c0dd</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, rclcpp::node_interfaces::NodeTimersInterface *node_timers)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~NodeTopics</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopics.html</anchorfile>
      <anchor>a3a3e555c6f7b1011252d7502d729cc00</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::PublisherBase::SharedPtr</type>
      <name>create_publisher</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopics.html</anchorfile>
      <anchor>afa2cbf8edb63bf25d67ded2e9e701056</anchor>
      <arglist>(const std::string &amp;topic_name, const rclcpp::PublisherFactory &amp;publisher_factory, const rclcpp::QoS &amp;qos) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_publisher</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopics.html</anchorfile>
      <anchor>a7e0983ccc5f3cda16a3f62710e6be7a5</anchor>
      <arglist>(rclcpp::PublisherBase::SharedPtr publisher, rclcpp::CallbackGroup::SharedPtr callback_group) override</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::SubscriptionBase::SharedPtr</type>
      <name>create_subscription</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopics.html</anchorfile>
      <anchor>a52942868df023cfcc8c32c43f0fefbd0</anchor>
      <arglist>(const std::string &amp;topic_name, const rclcpp::SubscriptionFactory &amp;subscription_factory, const rclcpp::QoS &amp;qos) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_subscription</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopics.html</anchorfile>
      <anchor>ae61f8555f125e83d0e3476c835e71cf2</anchor>
      <arglist>(rclcpp::SubscriptionBase::SharedPtr subscription, rclcpp::CallbackGroup::SharedPtr callback_group) override</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::node_interfaces::NodeBaseInterface *</type>
      <name>get_node_base_interface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopics.html</anchorfile>
      <anchor>a8cf573bbc0934a42c6d993781682f599</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::node_interfaces::NodeTimersInterface *</type>
      <name>get_node_timers_interface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopics.html</anchorfile>
      <anchor>a4fee7963aff007f509cc161da3c6fd31</anchor>
      <arglist>() const override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeTopicsInterface</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeTopicsInterface.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeTopicsInterface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopicsInterface.html</anchorfile>
      <anchor>a0f16a9a812977b32e9beb4ccd8bc8a13</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rclcpp::PublisherBase::SharedPtr</type>
      <name>create_publisher</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopicsInterface.html</anchorfile>
      <anchor>a603d741b92bef815eea5bc20245abda5</anchor>
      <arglist>(const std::string &amp;topic_name, const rclcpp::PublisherFactory &amp;publisher_factory, const rclcpp::QoS &amp;qos)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>add_publisher</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopicsInterface.html</anchorfile>
      <anchor>aca0caa0cf5d014a0b587449da085feb9</anchor>
      <arglist>(rclcpp::PublisherBase::SharedPtr publisher, rclcpp::CallbackGroup::SharedPtr callback_group)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rclcpp::SubscriptionBase::SharedPtr</type>
      <name>create_subscription</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopicsInterface.html</anchorfile>
      <anchor>a078ebe4f8599603e85f8aeb6bf9cf4d3</anchor>
      <arglist>(const std::string &amp;topic_name, const rclcpp::SubscriptionFactory &amp;subscription_factory, const rclcpp::QoS &amp;qos)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>add_subscription</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopicsInterface.html</anchorfile>
      <anchor>a8e51d66270927694c5d934870b522229</anchor>
      <arglist>(rclcpp::SubscriptionBase::SharedPtr subscription, rclcpp::CallbackGroup::SharedPtr callback_group)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rclcpp::node_interfaces::NodeBaseInterface *</type>
      <name>get_node_base_interface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopicsInterface.html</anchorfile>
      <anchor>a444c3da8c674eef25fac5fc82130fc6d</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual rclcpp::node_interfaces::NodeTimersInterface *</type>
      <name>get_node_timers_interface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeTopicsInterface.html</anchorfile>
      <anchor>a1e3c314baa2aa1d10f3a29b9bcb83be8</anchor>
      <arglist>() const =0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeWaitables</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeWaitables.html</filename>
    <base>rclcpp::node_interfaces::NodeWaitablesInterface</base>
    <member kind="function">
      <type></type>
      <name>NodeWaitables</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeWaitables.html</anchorfile>
      <anchor>a379499132e846d4dc757247a0738edc4</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeWaitables</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeWaitables.html</anchorfile>
      <anchor>a60e2225f1468bd770c201aac41e9224f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_waitable</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeWaitables.html</anchorfile>
      <anchor>a0c7df193f864d7438ebf960442560a9c</anchor>
      <arglist>(rclcpp::Waitable::SharedPtr waitable_base_ptr, rclcpp::CallbackGroup::SharedPtr group) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_waitable</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeWaitables.html</anchorfile>
      <anchor>a2f6b34d2672e5e00ef537bd720d8ebff</anchor>
      <arglist>(rclcpp::Waitable::SharedPtr waitable_ptr, rclcpp::CallbackGroup::SharedPtr group) noexcept override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::NodeWaitablesInterface</name>
    <filename>classrclcpp_1_1node__interfaces_1_1NodeWaitablesInterface.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~NodeWaitablesInterface</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeWaitablesInterface.html</anchorfile>
      <anchor>afc8d63eb52bc7e75455ef1f258fdb394</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>add_waitable</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeWaitablesInterface.html</anchorfile>
      <anchor>aa1300beb34151fb4fd5ec3e6d7fdbea8</anchor>
      <arglist>(rclcpp::Waitable::SharedPtr waitable_ptr, rclcpp::CallbackGroup::SharedPtr group)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>remove_waitable</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1NodeWaitablesInterface.html</anchorfile>
      <anchor>a0b8841473b65ef163edd02072c94929c</anchor>
      <arglist>(rclcpp::Waitable::SharedPtr waitable_ptr, rclcpp::CallbackGroup::SharedPtr group) noexcept=0</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::node_interfaces::OnSetParametersCallbackHandle</name>
    <filename>structrclcpp_1_1node__interfaces_1_1OnSetParametersCallbackHandle.html</filename>
    <member kind="typedef">
      <type>std::function&lt; rcl_interfaces::msg::SetParametersResult(const std::vector&lt; rclcpp::Parameter &gt; &amp;)&gt;</type>
      <name>OnParametersSetCallbackType</name>
      <anchorfile>structrclcpp_1_1node__interfaces_1_1OnSetParametersCallbackHandle.html</anchorfile>
      <anchor>a46979774780ee7d3d7be3ce0221701c0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>OnParametersSetCallbackType</type>
      <name>callback</name>
      <anchorfile>structrclcpp_1_1node__interfaces_1_1OnSetParametersCallbackHandle.html</anchorfile>
      <anchor>a8f85fe52291531e18cfd578697ee88d7</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Parameter</name>
    <filename>classrclcpp_1_1Parameter.html</filename>
    <member kind="function">
      <type></type>
      <name>Parameter</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a3a30798848ee315cdfe267ee23710755</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Parameter</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a2bc4a5f6e361662013d4807b634d51cd</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Parameter</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>af336f9865a97740ebd37e975566ee65c</anchor>
      <arglist>(const std::string &amp;name, const ParameterValue &amp;value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Parameter</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a19669e50d6d8056743da83963cc6b868</anchor>
      <arglist>(const std::string &amp;name, ValueTypeT value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Parameter</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a28ab398bb87a2e73c68147ec373da97a</anchor>
      <arglist>(const rclcpp::node_interfaces::ParameterInfo &amp;parameter_info)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a1290c58b13eb40cb90698a88aa4f1c02</anchor>
      <arglist>(const Parameter &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a0eea2292c5fdd7d4b4f62bc5888dfe0d</anchor>
      <arglist>(const Parameter &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>ParameterType</type>
      <name>get_type</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>ac845c383400a1264f168cca55daab110</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>get_type_name</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a6d46a8bfab751c646672c2063a54b2df</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>get_name</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>aa12dea07fc6adb5ef4d9ef4d8a1f5667</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rcl_interfaces::msg::ParameterValue</type>
      <name>get_value_message</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>acb43d6d13d8d641b50971ed039a1e6da</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const rclcpp::ParameterValue &amp;</type>
      <name>get_parameter_value</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>abc918bf8feb588c5a840cfb3f4e26382</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>decltype(auto)</type>
      <name>get_value</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a9042da271855312c63c3db86162c9b2a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>decltype(auto)</type>
      <name>get_value</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a9042da271855312c63c3db86162c9b2a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>as_bool</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a39a1e50ab9f4089bde624b8271dbc389</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>int64_t</type>
      <name>as_int</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>af890422a0bd00088a53b01a6e8ff2cad</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>as_double</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a498741c9117d2f5cb2de0a73f110a0bf</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>as_string</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a17764ccb8f486ce72ef735c7877b7b2e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; uint8_t &gt; &amp;</type>
      <name>as_byte_array</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a7562c9abba4c99c2e1e570ac68d11f12</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; bool &gt; &amp;</type>
      <name>as_bool_array</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a4ae8c4a4dfbaa300b741daa978bd8763</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; int64_t &gt; &amp;</type>
      <name>as_integer_array</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>af7575c8b1c49c8c4756a66d4cad3347f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; double &gt; &amp;</type>
      <name>as_double_array</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a19f4b64a1357efc8209fdec19c9ea048</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; std::string &gt; &amp;</type>
      <name>as_string_array</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a315585d1b51ef28fc635d885defe7813</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rcl_interfaces::msg::Parameter</type>
      <name>to_parameter_msg</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>a44dd983ac7c47bb5f0a9d7f946cca091</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>value_to_string</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>aaf2dc3ad81432a965443129abd370415</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static Parameter</type>
      <name>from_parameter_msg</name>
      <anchorfile>classrclcpp_1_1Parameter.html</anchorfile>
      <anchor>ade755f36f375f8c233d61624ace3d33f</anchor>
      <arglist>(const rcl_interfaces::msg::Parameter &amp;parameter)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::ParameterAlreadyDeclaredException</name>
    <filename>classrclcpp_1_1exceptions_1_1ParameterAlreadyDeclaredException.html</filename>
    <base>std::runtime_error</base>
  </compound>
  <compound kind="class">
    <name>rclcpp::ParameterEventsFilter</name>
    <filename>classrclcpp_1_1ParameterEventsFilter.html</filename>
    <member kind="enumeration">
      <type></type>
      <name>EventType</name>
      <anchorfile>classrclcpp_1_1ParameterEventsFilter.html</anchorfile>
      <anchor>a4234006c9ba27e23758b7bdc3501d3ea</anchor>
      <arglist></arglist>
      <enumvalue file="classrclcpp_1_1ParameterEventsFilter.html" anchor="a4234006c9ba27e23758b7bdc3501d3eaa24d459a81449d7210c8f9a86c2913034">NEW</enumvalue>
      <enumvalue file="classrclcpp_1_1ParameterEventsFilter.html" anchor="a4234006c9ba27e23758b7bdc3501d3eaa63c2867fdcae0e8e8413d7ac21b69b59">DELETED</enumvalue>
      <enumvalue file="classrclcpp_1_1ParameterEventsFilter.html" anchor="a4234006c9ba27e23758b7bdc3501d3eaae6b94e58bfd13b21bc786578d9f8ba4a">CHANGED</enumvalue>
    </member>
    <member kind="typedef">
      <type>std::pair&lt; EventType, rcl_interfaces::msg::Parameter * &gt;</type>
      <name>EventPair</name>
      <anchorfile>classrclcpp_1_1ParameterEventsFilter.html</anchorfile>
      <anchor>aa439f7163e32eb3b592c7f69732ea743</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterEventsFilter</name>
      <anchorfile>classrclcpp_1_1ParameterEventsFilter.html</anchorfile>
      <anchor>a8c2f391e4bd0057c1c87bc6907f41cbe</anchor>
      <arglist>(rcl_interfaces::msg::ParameterEvent::SharedPtr event, const std::vector&lt; std::string &gt; &amp;names, const std::vector&lt; EventType &gt; &amp;types)</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; EventPair &gt; &amp;</type>
      <name>get_events</name>
      <anchorfile>classrclcpp_1_1ParameterEventsFilter.html</anchorfile>
      <anchor>ae654625c85373a030d143ef904f1acb9</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::ParameterEventsQoS</name>
    <filename>classrclcpp_1_1ParameterEventsQoS.html</filename>
    <base>rclcpp::QoS</base>
    <member kind="function">
      <type></type>
      <name>ParameterEventsQoS</name>
      <anchorfile>classrclcpp_1_1ParameterEventsQoS.html</anchorfile>
      <anchor>a83e3dec7e1805ecbe3da76a31799cd1a</anchor>
      <arglist>(const QoSInitialization &amp;qos_initialization=(QoSInitialization::from_rmw(rmw_qos_profile_parameter_events)))</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::ParameterImmutableException</name>
    <filename>classrclcpp_1_1exceptions_1_1ParameterImmutableException.html</filename>
    <base>std::runtime_error</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::node_interfaces::ParameterInfo</name>
    <filename>structrclcpp_1_1node__interfaces_1_1ParameterInfo.html</filename>
    <member kind="variable">
      <type>rclcpp::ParameterValue</type>
      <name>value</name>
      <anchorfile>structrclcpp_1_1node__interfaces_1_1ParameterInfo.html</anchorfile>
      <anchor>aca2a24368f71199fee9af5371f95e2a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rcl_interfaces::msg::ParameterDescriptor</type>
      <name>descriptor</name>
      <anchorfile>structrclcpp_1_1node__interfaces_1_1ParameterInfo.html</anchorfile>
      <anchor>a9c062df47e4b6e3ef89ed591b09a071d</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::ParameterModifiedInCallbackException</name>
    <filename>classrclcpp_1_1exceptions_1_1ParameterModifiedInCallbackException.html</filename>
    <base>std::runtime_error</base>
  </compound>
  <compound kind="class">
    <name>rclcpp::node_interfaces::ParameterMutationRecursionGuard</name>
    <filename>classrclcpp_1_1node__interfaces_1_1ParameterMutationRecursionGuard.html</filename>
    <member kind="function">
      <type></type>
      <name>ParameterMutationRecursionGuard</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1ParameterMutationRecursionGuard.html</anchorfile>
      <anchor>a44dc8963a440d59c5e3afc2b0d26a8de</anchor>
      <arglist>(bool &amp;allow_mod)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ParameterMutationRecursionGuard</name>
      <anchorfile>classrclcpp_1_1node__interfaces_1_1ParameterMutationRecursionGuard.html</anchorfile>
      <anchor>a930ff5d58fb22849b2dbbf4a1a3cc1ec</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::ParameterNotDeclaredException</name>
    <filename>classrclcpp_1_1exceptions_1_1ParameterNotDeclaredException.html</filename>
    <base>std::runtime_error</base>
  </compound>
  <compound kind="class">
    <name>rclcpp::ParameterService</name>
    <filename>classrclcpp_1_1ParameterService.html</filename>
    <member kind="function">
      <type></type>
      <name>ParameterService</name>
      <anchorfile>classrclcpp_1_1ParameterService.html</anchorfile>
      <anchor>af35d5f9850a349c2977e0f6a48af7ca3</anchor>
      <arglist>(const std::shared_ptr&lt; node_interfaces::NodeBaseInterface &gt; node_base, const std::shared_ptr&lt; node_interfaces::NodeServicesInterface &gt; node_services, rclcpp::node_interfaces::NodeParametersInterface *node_params, const rmw_qos_profile_t &amp;qos_profile=rmw_qos_profile_parameters)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::ParametersQoS</name>
    <filename>classrclcpp_1_1ParametersQoS.html</filename>
    <base>rclcpp::QoS</base>
    <member kind="function">
      <type></type>
      <name>ParametersQoS</name>
      <anchorfile>classrclcpp_1_1ParametersQoS.html</anchorfile>
      <anchor>ade3d42ce08011ccdc6e94b86ac265eb4</anchor>
      <arglist>(const QoSInitialization &amp;qos_initialization=(QoSInitialization::from_rmw(rmw_qos_profile_parameters)))</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::ParameterTypeException</name>
    <filename>classrclcpp_1_1ParameterTypeException.html</filename>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>ParameterTypeException</name>
      <anchorfile>classrclcpp_1_1ParameterTypeException.html</anchorfile>
      <anchor>a5f14615ace48474c74f795447e694113</anchor>
      <arglist>(ParameterType expected, ParameterType actual)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::ParameterValue</name>
    <filename>classrclcpp_1_1ParameterValue.html</filename>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>ada4e33aaa0b4807cb2571ce9c4b5a397</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a9b97ef33e763c734e09ecece798df132</anchor>
      <arglist>(const rcl_interfaces::msg::ParameterValue &amp;value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>aeced39a7d72c822bd6adc39d4f200e19</anchor>
      <arglist>(const bool bool_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a682c6b0ec3c1dfae0a0421cdea036e59</anchor>
      <arglist>(const int int_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>ad4823752011957ea892637c95ec63dbe</anchor>
      <arglist>(const int64_t int_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a5b34b18860c8b4bbaa1a8a6a0eee09df</anchor>
      <arglist>(const float double_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a92448a5b0df56147e741511bd2572717</anchor>
      <arglist>(const double double_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>af9a64d287393b1a6ad4242c9634b03aa</anchor>
      <arglist>(const std::string &amp;string_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a3a7177eb1e5c049f9233f748cf3955b6</anchor>
      <arglist>(const char *string_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>afe4cf4dcc0a3bc2c72454407a078c41a</anchor>
      <arglist>(const std::vector&lt; uint8_t &gt; &amp;byte_array_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>ab48b2e48fadec2a85079d46b61e93ee6</anchor>
      <arglist>(const std::vector&lt; bool &gt; &amp;bool_array_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a55282e5bbc47ebf5cd8370f2b930c1ab</anchor>
      <arglist>(const std::vector&lt; int &gt; &amp;int_array_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a313d8d4e4343853e96571dceab959734</anchor>
      <arglist>(const std::vector&lt; int64_t &gt; &amp;int_array_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a3a77d5285d620899b0a7e0efe5042123</anchor>
      <arglist>(const std::vector&lt; float &gt; &amp;double_array_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a7803525436ca69d7338563f0897d9687</anchor>
      <arglist>(const std::vector&lt; double &gt; &amp;double_array_value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ParameterValue</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a21b4661360eabdaf37575c57bdd84b73</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;string_array_value)</arglist>
    </member>
    <member kind="function">
      <type>ParameterType</type>
      <name>get_type</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a3ad22b2fca1918fae948dc17b03bbc63</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rcl_interfaces::msg::ParameterValue</type>
      <name>to_value_msg</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>afcb23823b668bb849e124893e825c698</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a53a9c161f8af8a5d4b219160cfd87655</anchor>
      <arglist>(const ParameterValue &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a1e3219da506abc748d2dfc86067870bc</anchor>
      <arglist>(const ParameterValue &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; type==ParameterType::PARAMETER_BOOL, const bool &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>ab0e09fc108346c2265dee533166f9805</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; type==ParameterType::PARAMETER_INTEGER, const int64_t &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>ae977095ea0178a672e5ee4a382c595e8</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; type==ParameterType::PARAMETER_DOUBLE, const double &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a76b1e4281e48a87f78078123c5ec34d8</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; type==ParameterType::PARAMETER_STRING, const std::string &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a6ac25e3f70fd596c909e081c56621a87</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; type==ParameterType::PARAMETER_BYTE_ARRAY, const std::vector&lt; uint8_t &gt; &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a4914071e55423fd7ae25cd8c532ceafa</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; type==ParameterType::PARAMETER_BOOL_ARRAY, const std::vector&lt; bool &gt; &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>abf8dada9ab749a42d4ae08059981b258</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; type==ParameterType::PARAMETER_INTEGER_ARRAY, const std::vector&lt; int64_t &gt; &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a59a385f736a19a0aa237d9384737a59e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; type==ParameterType::PARAMETER_DOUBLE_ARRAY, const std::vector&lt; double &gt; &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a28c534126126fc215468bbd403911929</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; type==ParameterType::PARAMETER_STRING_ARRAY, const std::vector&lt; std::string &gt; &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>ac25417f0938fff6165228f993d311552</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; std::is_same&lt; type, bool &gt;::value, const bool &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a8e9949df39f2a24187a86339be4fe6f5</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; std::is_integral&lt; type &gt;::value &amp;&amp;!std::is_same&lt; type, bool &gt;::value, const int64_t &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a72236d0062e2e719db0852751cb1175e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; std::is_floating_point&lt; type &gt;::value, const double &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>ae2180162a5abe1d3cf6c35a135d3e366</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; std::is_convertible&lt; type, std::string &gt;::value, const std::string &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a7f8ad7b97d9a42bc59e96bd2526e620e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; std::is_convertible&lt; type, const std::vector&lt; uint8_t &gt; &amp; &gt;::value, const std::vector&lt; uint8_t &gt; &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>add3c0992110c04394346f66de2eba357</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; std::is_convertible&lt; type, const std::vector&lt; bool &gt; &amp; &gt;::value, const std::vector&lt; bool &gt; &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>aa84b887bd85587160a1f857d3d8a4979</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; std::is_convertible&lt; type, const std::vector&lt; int64_t &gt; &amp; &gt;::value, const std::vector&lt; int64_t &gt; &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a6f1cacad493079fbda149ff85756a56e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; std::is_convertible&lt; type, const std::vector&lt; double &gt; &amp; &gt;::value, const std::vector&lt; double &gt; &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a4665bbee46015f219dd1841404c4f568</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::enable_if&lt; std::is_convertible&lt; type, const std::vector&lt; std::string &gt; &amp; &gt;::value, const std::vector&lt; std::string &gt; &amp; &gt;::type</type>
      <name>get</name>
      <anchorfile>classrclcpp_1_1ParameterValue.html</anchorfile>
      <anchor>a57bd27d097e83ed7f80d523cece4c1ba</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy::PoolMember</name>
    <filename>structrclcpp_1_1strategies_1_1message__pool__memory__strategy_1_1MessagePoolMemoryStrategy_1_1PoolMember.html</filename>
    <member kind="variable">
      <type>std::shared_ptr&lt; MessageT &gt;</type>
      <name>msg_ptr_</name>
      <anchorfile>structrclcpp_1_1strategies_1_1message__pool__memory__strategy_1_1MessagePoolMemoryStrategy_1_1PoolMember.html</anchorfile>
      <anchor>a7d3345e53017d13f429fc1fc7ac9a6a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>used</name>
      <anchorfile>structrclcpp_1_1strategies_1_1message__pool__memory__strategy_1_1MessagePoolMemoryStrategy_1_1PoolMember.html</anchorfile>
      <anchor>aabe1065d4987465c0d5000619147dd05</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Publisher</name>
    <filename>classrclcpp_1_1Publisher.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <base>rclcpp::PublisherBase</base>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; MessageT, AllocatorT &gt;</type>
      <name>MessageAllocatorTraits</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a9aea4a053601d9979f82ce3b76f6b159</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename MessageAllocatorTraits::allocator_type</type>
      <name>MessageAllocator</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>aa141d169a9c7265220b198ffe21c485c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; MessageAllocator, MessageT &gt;</type>
      <name>MessageDeleter</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a015c20710fc2665d22c72ef8c4d2e016</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::unique_ptr&lt; MessageT, MessageDeleter &gt;</type>
      <name>MessageUniquePtr</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a75b69f055c750997df6dc82d56ccb1ff</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; const MessageT &gt;</type>
      <name>MessageSharedPtr</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a8f667265c0f4786d234fc227ac86c989</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Publisher</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>addc5cfc67aef705c42f236b46b748abf</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, const std::string &amp;topic, const rclcpp::QoS &amp;qos, const rclcpp::PublisherOptionsWithAllocator&lt; AllocatorT &gt; &amp;options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>post_init_setup</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a1fb33f9786843d35df5f0e1d58f5c78e</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, const std::string &amp;topic, const rclcpp::QoS &amp;qos, const rclcpp::PublisherOptionsWithAllocator&lt; AllocatorT &gt; &amp;options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Publisher</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a008b9a8976e93d4ffeeeb5e9293a1136</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::LoanedMessage&lt; MessageT, AllocatorT &gt;</type>
      <name>borrow_loaned_message</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a610961210375964bafc0d6bdb61164ca</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>aace16d5f3e3c1c533fd0098602613a29</anchor>
      <arglist>(std::unique_ptr&lt; MessageT, MessageDeleter &gt; msg)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>ad517601ae3bc33b139b3a44e81b04898</anchor>
      <arglist>(const MessageT &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a220ff5e23791adf9fc0ece8b456b4881</anchor>
      <arglist>(const rcl_serialized_message_t &amp;serialized_msg)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a10b5ec669fe9dd1f3e1034dc54c88289</anchor>
      <arglist>(const SerializedMessage &amp;serialized_msg)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>ac452c182c10c91b8b8b120d60ffd2fb3</anchor>
      <arglist>(rclcpp::LoanedMessage&lt; MessageT, AllocatorT &gt; &amp;&amp;loaned_msg)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; MessageAllocator &gt;</type>
      <name>get_allocator</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a2f4ca7b4f047d3d58991b6b10d940cf0</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>do_inter_process_publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a881e4ef60a4d67059419668a6644ff61</anchor>
      <arglist>(const MessageT &amp;msg)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>do_serialized_publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>af52d4691f23e658cd153f0285f194068</anchor>
      <arglist>(const rcl_serialized_message_t *serialized_msg)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>do_loaned_message_publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>ac4aa7b7b9f7d868b2f2224ad87415fe5</anchor>
      <arglist>(MessageT *msg)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>do_intra_process_publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>aebe6c79c9d9aca0f58669232c29312b7</anchor>
      <arglist>(std::unique_ptr&lt; MessageT, MessageDeleter &gt; msg)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::shared_ptr&lt; const MessageT &gt;</type>
      <name>do_intra_process_publish_and_return_shared</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>ad78992eb2fa941c1594c48b4d650a86f</anchor>
      <arglist>(std::unique_ptr&lt; MessageT, MessageDeleter &gt; msg)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const rclcpp::PublisherOptionsWithAllocator&lt; AllocatorT &gt;</type>
      <name>options_</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a6765e91feeefc7a58575070deecfc261</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; MessageAllocator &gt;</type>
      <name>message_allocator_</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a406e6464ee0bc9a42bc99492a74d612d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>MessageDeleter</type>
      <name>message_deleter_</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a6121226adfc64f58320d5dccd5110745</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Publisher&lt; rcl_interfaces::msg::ParameterEvent &gt;</name>
    <filename>classrclcpp_1_1Publisher.html</filename>
    <base>rclcpp::PublisherBase</base>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; rcl_interfaces::msg::ParameterEvent, std::allocator&lt; void &gt; &gt;</type>
      <name>MessageAllocatorTraits</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a9aea4a053601d9979f82ce3b76f6b159</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename MessageAllocatorTraits::allocator_type</type>
      <name>MessageAllocator</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>aa141d169a9c7265220b198ffe21c485c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; MessageAllocator, rcl_interfaces::msg::ParameterEvent &gt;</type>
      <name>MessageDeleter</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a015c20710fc2665d22c72ef8c4d2e016</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::unique_ptr&lt; rcl_interfaces::msg::ParameterEvent, MessageDeleter &gt;</type>
      <name>MessageUniquePtr</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a75b69f055c750997df6dc82d56ccb1ff</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; const rcl_interfaces::msg::ParameterEvent &gt;</type>
      <name>MessageSharedPtr</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a8f667265c0f4786d234fc227ac86c989</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Publisher</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>addc5cfc67aef705c42f236b46b748abf</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, const std::string &amp;topic, const rclcpp::QoS &amp;qos, const rclcpp::PublisherOptionsWithAllocator&lt; std::allocator&lt; void &gt; &gt; &amp;options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>post_init_setup</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a1fb33f9786843d35df5f0e1d58f5c78e</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, const std::string &amp;topic, const rclcpp::QoS &amp;qos, const rclcpp::PublisherOptionsWithAllocator&lt; std::allocator&lt; void &gt; &gt; &amp;options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Publisher</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a008b9a8976e93d4ffeeeb5e9293a1136</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::LoanedMessage&lt; rcl_interfaces::msg::ParameterEvent, std::allocator&lt; void &gt; &gt;</type>
      <name>borrow_loaned_message</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a610961210375964bafc0d6bdb61164ca</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>aace16d5f3e3c1c533fd0098602613a29</anchor>
      <arglist>(std::unique_ptr&lt; rcl_interfaces::msg::ParameterEvent, MessageDeleter &gt; msg)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>ad517601ae3bc33b139b3a44e81b04898</anchor>
      <arglist>(const rcl_interfaces::msg::ParameterEvent &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a220ff5e23791adf9fc0ece8b456b4881</anchor>
      <arglist>(const rcl_serialized_message_t &amp;serialized_msg)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a10b5ec669fe9dd1f3e1034dc54c88289</anchor>
      <arglist>(const SerializedMessage &amp;serialized_msg)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>ac452c182c10c91b8b8b120d60ffd2fb3</anchor>
      <arglist>(rclcpp::LoanedMessage&lt; rcl_interfaces::msg::ParameterEvent, std::allocator&lt; void &gt; &gt; &amp;&amp;loaned_msg)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; MessageAllocator &gt;</type>
      <name>get_allocator</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a2f4ca7b4f047d3d58991b6b10d940cf0</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>do_inter_process_publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a881e4ef60a4d67059419668a6644ff61</anchor>
      <arglist>(const rcl_interfaces::msg::ParameterEvent &amp;msg)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>do_serialized_publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>af52d4691f23e658cd153f0285f194068</anchor>
      <arglist>(const rcl_serialized_message_t *serialized_msg)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>do_loaned_message_publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>ac4aa7b7b9f7d868b2f2224ad87415fe5</anchor>
      <arglist>(rcl_interfaces::msg::ParameterEvent *msg)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>do_intra_process_publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>aebe6c79c9d9aca0f58669232c29312b7</anchor>
      <arglist>(std::unique_ptr&lt; rcl_interfaces::msg::ParameterEvent, MessageDeleter &gt; msg)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::shared_ptr&lt; const rcl_interfaces::msg::ParameterEvent &gt;</type>
      <name>do_intra_process_publish_and_return_shared</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>ad78992eb2fa941c1594c48b4d650a86f</anchor>
      <arglist>(std::unique_ptr&lt; rcl_interfaces::msg::ParameterEvent, MessageDeleter &gt; msg)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const rclcpp::PublisherOptionsWithAllocator&lt; std::allocator&lt; void &gt; &gt;</type>
      <name>options_</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a6765e91feeefc7a58575070deecfc261</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; MessageAllocator &gt;</type>
      <name>message_allocator_</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a406e6464ee0bc9a42bc99492a74d612d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>MessageDeleter</type>
      <name>message_deleter_</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a6121226adfc64f58320d5dccd5110745</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Publisher&lt; statistics_msgs::msg::MetricsMessage &gt;</name>
    <filename>classrclcpp_1_1Publisher.html</filename>
    <base>rclcpp::PublisherBase</base>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; statistics_msgs::msg::MetricsMessage, std::allocator&lt; void &gt; &gt;</type>
      <name>MessageAllocatorTraits</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a9aea4a053601d9979f82ce3b76f6b159</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename MessageAllocatorTraits::allocator_type</type>
      <name>MessageAllocator</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>aa141d169a9c7265220b198ffe21c485c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; MessageAllocator, statistics_msgs::msg::MetricsMessage &gt;</type>
      <name>MessageDeleter</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a015c20710fc2665d22c72ef8c4d2e016</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::unique_ptr&lt; statistics_msgs::msg::MetricsMessage, MessageDeleter &gt;</type>
      <name>MessageUniquePtr</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a75b69f055c750997df6dc82d56ccb1ff</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; const statistics_msgs::msg::MetricsMessage &gt;</type>
      <name>MessageSharedPtr</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a8f667265c0f4786d234fc227ac86c989</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Publisher</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>addc5cfc67aef705c42f236b46b748abf</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, const std::string &amp;topic, const rclcpp::QoS &amp;qos, const rclcpp::PublisherOptionsWithAllocator&lt; std::allocator&lt; void &gt; &gt; &amp;options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>post_init_setup</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a1fb33f9786843d35df5f0e1d58f5c78e</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, const std::string &amp;topic, const rclcpp::QoS &amp;qos, const rclcpp::PublisherOptionsWithAllocator&lt; std::allocator&lt; void &gt; &gt; &amp;options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Publisher</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a008b9a8976e93d4ffeeeb5e9293a1136</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::LoanedMessage&lt; statistics_msgs::msg::MetricsMessage, std::allocator&lt; void &gt; &gt;</type>
      <name>borrow_loaned_message</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a610961210375964bafc0d6bdb61164ca</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>aace16d5f3e3c1c533fd0098602613a29</anchor>
      <arglist>(std::unique_ptr&lt; statistics_msgs::msg::MetricsMessage, MessageDeleter &gt; msg)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>ad517601ae3bc33b139b3a44e81b04898</anchor>
      <arglist>(const statistics_msgs::msg::MetricsMessage &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a220ff5e23791adf9fc0ece8b456b4881</anchor>
      <arglist>(const rcl_serialized_message_t &amp;serialized_msg)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a10b5ec669fe9dd1f3e1034dc54c88289</anchor>
      <arglist>(const SerializedMessage &amp;serialized_msg)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>ac452c182c10c91b8b8b120d60ffd2fb3</anchor>
      <arglist>(rclcpp::LoanedMessage&lt; statistics_msgs::msg::MetricsMessage, std::allocator&lt; void &gt; &gt; &amp;&amp;loaned_msg)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; MessageAllocator &gt;</type>
      <name>get_allocator</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a2f4ca7b4f047d3d58991b6b10d940cf0</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>do_inter_process_publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a881e4ef60a4d67059419668a6644ff61</anchor>
      <arglist>(const statistics_msgs::msg::MetricsMessage &amp;msg)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>do_serialized_publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>af52d4691f23e658cd153f0285f194068</anchor>
      <arglist>(const rcl_serialized_message_t *serialized_msg)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>do_loaned_message_publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>ac4aa7b7b9f7d868b2f2224ad87415fe5</anchor>
      <arglist>(statistics_msgs::msg::MetricsMessage *msg)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>do_intra_process_publish</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>aebe6c79c9d9aca0f58669232c29312b7</anchor>
      <arglist>(std::unique_ptr&lt; statistics_msgs::msg::MetricsMessage, MessageDeleter &gt; msg)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::shared_ptr&lt; const statistics_msgs::msg::MetricsMessage &gt;</type>
      <name>do_intra_process_publish_and_return_shared</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>ad78992eb2fa941c1594c48b4d650a86f</anchor>
      <arglist>(std::unique_ptr&lt; statistics_msgs::msg::MetricsMessage, MessageDeleter &gt; msg)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const rclcpp::PublisherOptionsWithAllocator&lt; std::allocator&lt; void &gt; &gt;</type>
      <name>options_</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a6765e91feeefc7a58575070deecfc261</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; MessageAllocator &gt;</type>
      <name>message_allocator_</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a406e6464ee0bc9a42bc99492a74d612d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>MessageDeleter</type>
      <name>message_deleter_</name>
      <anchorfile>classrclcpp_1_1Publisher.html</anchorfile>
      <anchor>a6121226adfc64f58320d5dccd5110745</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::PublisherBase</name>
    <filename>classrclcpp_1_1PublisherBase.html</filename>
    <base>enable_shared_from_this&lt; PublisherBase &gt;</base>
    <member kind="typedef">
      <type>std::shared_ptr&lt; rclcpp::experimental::IntraProcessManager &gt;</type>
      <name>IntraProcessManagerSharedPtr</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a9ffed540145a53715b15f81d38f5bc8b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>PublisherBase</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a66fb8913861f0d01fab99f73a956b7de</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, const std::string &amp;topic, const rosidl_message_type_support_t &amp;type_support, const rcl_publisher_options_t &amp;publisher_options)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~PublisherBase</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>ac701862a17e617444699522640018cb0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_topic_name</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a24ff8fa35002c746560a43e5bed4d1ac</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_queue_size</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a3b558a91b17553b9a73be6df2b1392bd</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const rmw_gid_t &amp;</type>
      <name>get_gid</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a9babfc90be49163805f177e22867e742</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rcl_publisher_t &gt;</type>
      <name>get_publisher_handle</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a5048a5bec0416925e17f31b883163d25</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; const rcl_publisher_t &gt;</type>
      <name>get_publisher_handle</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a4491d6dd37264815c80b9ac2970975e0</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; std::shared_ptr&lt; rclcpp::QOSEventHandlerBase &gt; &gt; &amp;</type>
      <name>get_event_handlers</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>ac53baf0269f5132c35b3752bac6c7412</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_subscription_count</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a60c84260b2ff0fb364fad688ed91234f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_intra_process_subscription_count</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a4425b7c067368f9271a7761c9ad2a05b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>RCUTILS_WARN_UNUSED bool</type>
      <name>assert_liveliness</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>ad38e19dc3110a0683047feef868137ff</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::QoS</type>
      <name>get_actual_qos</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>ae646430342d6142e6a7335c0487ce913</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>can_loan_messages</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>af2dc895860fc6ab8d851f8ceaaa71b8e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a1ce90a99a8ae4ee4de55f50e71eb86ba</anchor>
      <arglist>(const rmw_gid_t &amp;gid) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a56b975ea146088721ea3876d880d7d97</anchor>
      <arglist>(const rmw_gid_t *gid) const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setup_intra_process</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a178cd03f60d78042f47e26db9682c585</anchor>
      <arglist>(uint64_t intra_process_publisher_id, IntraProcessManagerSharedPtr ipm)</arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::weak_ptr&lt; rclcpp::experimental::IntraProcessManager &gt;</type>
      <name>IntraProcessManagerWeakPtr</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a7e761197624044b248314a40827e55b0</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>add_event_handler</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>ad84eedef6a8269f553551296cbf41b34</anchor>
      <arglist>(const EventCallbackT &amp;callback, const rcl_publisher_event_type_t event_type)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>default_incompatible_qos_callback</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a2e1803c62b437626f125f997e1d79028</anchor>
      <arglist>(QOSOfferedIncompatibleQoSInfo &amp;info) const</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rcl_node_t &gt;</type>
      <name>rcl_node_handle_</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a495a7c3aab79046f9ba43412df0b0e07</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rcl_publisher_t &gt;</type>
      <name>publisher_handle_</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a5540f1c25d6cf177f9dcd942967c0a84</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; std::shared_ptr&lt; rclcpp::QOSEventHandlerBase &gt; &gt;</type>
      <name>event_handlers_</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a97df58ad184e7edc892fb155486a9baa</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>bool</type>
      <name>intra_process_is_enabled_</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a107a84e5dcd173574dc25124045fd4a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>IntraProcessManagerWeakPtr</type>
      <name>weak_ipm_</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a514d719eb64eaa2a3e91a08da82dab26</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>uint64_t</type>
      <name>intra_process_publisher_id_</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>aabbae7802b05b6563da0640d8af5e8c6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rmw_gid_t</type>
      <name>rmw_gid_</name>
      <anchorfile>classrclcpp_1_1PublisherBase.html</anchorfile>
      <anchor>a19e12471a0b5981fa5b84885b4bc3260</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::PublisherEventCallbacks</name>
    <filename>structrclcpp_1_1PublisherEventCallbacks.html</filename>
    <member kind="variable">
      <type>QOSDeadlineOfferedCallbackType</type>
      <name>deadline_callback</name>
      <anchorfile>structrclcpp_1_1PublisherEventCallbacks.html</anchorfile>
      <anchor>aea23d69d4b8dbfe1a444bc5b56f0d059</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>QOSLivelinessLostCallbackType</type>
      <name>liveliness_callback</name>
      <anchorfile>structrclcpp_1_1PublisherEventCallbacks.html</anchorfile>
      <anchor>adf569256e7fb4f164b9d404676fc1ce3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>QOSOfferedIncompatibleQoSCallbackType</type>
      <name>incompatible_qos_callback</name>
      <anchorfile>structrclcpp_1_1PublisherEventCallbacks.html</anchorfile>
      <anchor>ad31ee8e2db7259bc4dea2039125b79c6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::PublisherFactory</name>
    <filename>structrclcpp_1_1PublisherFactory.html</filename>
    <member kind="typedef">
      <type>std::function&lt; rclcpp::PublisherBase::SharedPtr(rclcpp::node_interfaces::NodeBaseInterface *node_base, const std::string &amp;topic_name, const rclcpp::QoS &amp;qos)&gt;</type>
      <name>PublisherFactoryFunction</name>
      <anchorfile>structrclcpp_1_1PublisherFactory.html</anchorfile>
      <anchor>a7f84d97d02f5865105a9466b6f306b2c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const PublisherFactoryFunction</type>
      <name>create_typed_publisher</name>
      <anchorfile>structrclcpp_1_1PublisherFactory.html</anchorfile>
      <anchor>a28678b28d3d2b13983e282fbd3356463</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::PublisherOptionsBase</name>
    <filename>structrclcpp_1_1PublisherOptionsBase.html</filename>
    <member kind="variable">
      <type>IntraProcessSetting</type>
      <name>use_intra_process_comm</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsBase.html</anchorfile>
      <anchor>abaee5351c2f21430e06f4f35b22a4bf1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>PublisherEventCallbacks</type>
      <name>event_callbacks</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsBase.html</anchorfile>
      <anchor>a51abfcf09886555e9cb97421d8d4019b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>use_default_callbacks</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsBase.html</anchorfile>
      <anchor>a9d2ebb15f86a711fb2611e48e9fa5251</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; rclcpp::CallbackGroup &gt;</type>
      <name>callback_group</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsBase.html</anchorfile>
      <anchor>adb8ddf726c69c049b15305520ae27c2c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; rclcpp::detail::RMWImplementationSpecificPublisherPayload &gt;</type>
      <name>rmw_implementation_payload</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsBase.html</anchorfile>
      <anchor>aa306652990cdd5d3790206cabd0b9d96</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::PublisherOptionsWithAllocator</name>
    <filename>structrclcpp_1_1PublisherOptionsWithAllocator.html</filename>
    <templarg></templarg>
    <base>rclcpp::PublisherOptionsBase</base>
    <member kind="function">
      <type></type>
      <name>PublisherOptionsWithAllocator</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsWithAllocator.html</anchorfile>
      <anchor>a677edf16b4b958ad369441f3755e960b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>PublisherOptionsWithAllocator</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsWithAllocator.html</anchorfile>
      <anchor>a18724bead67c0ce0c29334c85b545fb3</anchor>
      <arglist>(const PublisherOptionsBase &amp;publisher_options_base)</arglist>
    </member>
    <member kind="function">
      <type>rcl_publisher_options_t</type>
      <name>to_rcl_publisher_options</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsWithAllocator.html</anchorfile>
      <anchor>adb79b289e90213cc8349af42fe674caf</anchor>
      <arglist>(const rclcpp::QoS &amp;qos) const</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; Allocator &gt;</type>
      <name>get_allocator</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsWithAllocator.html</anchorfile>
      <anchor>a4d3138df42edcf99ade44ea8f6998d57</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; Allocator &gt;</type>
      <name>allocator</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsWithAllocator.html</anchorfile>
      <anchor>a3fc3a64dd2be630c3a3c387a021ceb1e</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>PublisherOptionsWithAllocator&lt; std::allocator&lt; void &gt; &gt;</name>
    <filename>structrclcpp_1_1PublisherOptionsWithAllocator.html</filename>
    <base>rclcpp::PublisherOptionsBase</base>
    <member kind="function">
      <type></type>
      <name>PublisherOptionsWithAllocator</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsWithAllocator.html</anchorfile>
      <anchor>a677edf16b4b958ad369441f3755e960b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>PublisherOptionsWithAllocator</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsWithAllocator.html</anchorfile>
      <anchor>a18724bead67c0ce0c29334c85b545fb3</anchor>
      <arglist>(const PublisherOptionsBase &amp;publisher_options_base)</arglist>
    </member>
    <member kind="function">
      <type>rcl_publisher_options_t</type>
      <name>to_rcl_publisher_options</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsWithAllocator.html</anchorfile>
      <anchor>adb79b289e90213cc8349af42fe674caf</anchor>
      <arglist>(const rclcpp::QoS &amp;qos) const</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; std::allocator&lt; void &gt; &gt;</type>
      <name>get_allocator</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsWithAllocator.html</anchorfile>
      <anchor>a4d3138df42edcf99ade44ea8f6998d57</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; std::allocator&lt; void &gt; &gt;</type>
      <name>allocator</name>
      <anchorfile>structrclcpp_1_1PublisherOptionsWithAllocator.html</anchorfile>
      <anchor>a3fc3a64dd2be630c3a3c387a021ceb1e</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::QoS</name>
    <filename>classrclcpp_1_1QoS.html</filename>
    <member kind="function">
      <type></type>
      <name>QoS</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a98fb6b31d7c5cbd4788412663fd38cfb</anchor>
      <arglist>(const QoSInitialization &amp;qos_initialization, const rmw_qos_profile_t &amp;initial_profile=rmw_qos_profile_default)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>QoS</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>ad7e932d8e2f636c80eff674546ec3963</anchor>
      <arglist>(size_t history_depth)</arglist>
    </member>
    <member kind="function">
      <type>rmw_qos_profile_t &amp;</type>
      <name>get_rmw_qos_profile</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>abe5cb7d1ee2f6874e03f0c02937a3194</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rmw_qos_profile_t &amp;</type>
      <name>get_rmw_qos_profile</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>ad4f6afcb0793ce1370f43ebdd315e311</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>history</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a2744f88809277c859f47b0c84e6c916f</anchor>
      <arglist>(rmw_qos_history_policy_t history)</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>keep_last</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a21f3d3a262f23011ef9fbd05a22f1c1c</anchor>
      <arglist>(size_t depth)</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>keep_all</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a58be89f44939d2a6ac3a679e8c3097cf</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>reliability</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a5d9289ea7d507091d9bc72699a95dda2</anchor>
      <arglist>(rmw_qos_reliability_policy_t reliability)</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>reliable</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>acf09bbc275f571c3ace9ef4f7b507316</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>best_effort</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a6b7c622bf75770ad7ae18223e068cd9f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>durability</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a45f337395443d10fc19faa546439445f</anchor>
      <arglist>(rmw_qos_durability_policy_t durability)</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>durability_volatile</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a2db9e43edec109ec856f54685a096137</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>transient_local</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>add623fdda9ad41e273af48bb1813a8f5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>deadline</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a6c723dcfcae694ae6fd15c9b5707ad96</anchor>
      <arglist>(rmw_time_t deadline)</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>deadline</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>ad4e220bb52d4110de191d483ad36aca1</anchor>
      <arglist>(const rclcpp::Duration &amp;deadline)</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>lifespan</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a614ad81f728e86636b403a84da9ca5dd</anchor>
      <arglist>(rmw_time_t lifespan)</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>lifespan</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a80b69c7993aa02fbe13d23743f97324e</anchor>
      <arglist>(const rclcpp::Duration &amp;lifespan)</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>liveliness</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>ae46129faa89aca23ae714fb3ef243fa1</anchor>
      <arglist>(rmw_qos_liveliness_policy_t liveliness)</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>liveliness_lease_duration</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a3d2faaff7d4fddf4d47aa7c6d71b63a0</anchor>
      <arglist>(rmw_time_t liveliness_lease_duration)</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>liveliness_lease_duration</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>a6a4c870bbea189349a99c29f0dd1e4fb</anchor>
      <arglist>(const rclcpp::Duration &amp;liveliness_lease_duration)</arglist>
    </member>
    <member kind="function">
      <type>QoS &amp;</type>
      <name>avoid_ros_namespace_conventions</name>
      <anchorfile>classrclcpp_1_1QoS.html</anchorfile>
      <anchor>ace03ab9bda9662ef5159b2168a1548ec</anchor>
      <arglist>(bool avoid_ros_namespace_conventions)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::QOSEventHandler</name>
    <filename>classrclcpp_1_1QOSEventHandler.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <base>rclcpp::QOSEventHandlerBase</base>
    <member kind="function">
      <type></type>
      <name>QOSEventHandler</name>
      <anchorfile>classrclcpp_1_1QOSEventHandler.html</anchorfile>
      <anchor>afe4871d05d68c8c3d67cfb6187e93b25</anchor>
      <arglist>(const EventCallbackT &amp;callback, InitFuncT init_func, ParentHandleT parent_handle, EventTypeEnum event_type)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>execute</name>
      <anchorfile>classrclcpp_1_1QOSEventHandler.html</anchorfile>
      <anchor>ab1bfba2d6fb46e54dd5ab03e11f97643</anchor>
      <arglist>() override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::QOSEventHandlerBase</name>
    <filename>classrclcpp_1_1QOSEventHandlerBase.html</filename>
    <base>rclcpp::Waitable</base>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~QOSEventHandlerBase</name>
      <anchorfile>classrclcpp_1_1QOSEventHandlerBase.html</anchorfile>
      <anchor>aaf08f634aab6f805627327b4c01891a2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_number_of_ready_events</name>
      <anchorfile>classrclcpp_1_1QOSEventHandlerBase.html</anchorfile>
      <anchor>ab5dfa3202c159b8bb472da202eeb4351</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>add_to_wait_set</name>
      <anchorfile>classrclcpp_1_1QOSEventHandlerBase.html</anchorfile>
      <anchor>a835c269be1f6871a2da3a2cc6e68e9a4</anchor>
      <arglist>(rcl_wait_set_t *wait_set) override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_ready</name>
      <anchorfile>classrclcpp_1_1QOSEventHandlerBase.html</anchorfile>
      <anchor>a5c49178629eb341b095c144546ab5a25</anchor>
      <arglist>(rcl_wait_set_t *wait_set) override</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rcl_event_t</type>
      <name>event_handle_</name>
      <anchorfile>classrclcpp_1_1QOSEventHandlerBase.html</anchorfile>
      <anchor>afb5e361d5e17b1b7cad0276b17cdcb81</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>size_t</type>
      <name>wait_set_event_index_</name>
      <anchorfile>classrclcpp_1_1QOSEventHandlerBase.html</anchorfile>
      <anchor>ac5a7540ffe0732239579cbf4468c0e77</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::QoSInitialization</name>
    <filename>structrclcpp_1_1QoSInitialization.html</filename>
    <member kind="function">
      <type></type>
      <name>QoSInitialization</name>
      <anchorfile>structrclcpp_1_1QoSInitialization.html</anchorfile>
      <anchor>a94456d03c20cab02fbe8a4b620c17b2f</anchor>
      <arglist>(rmw_qos_history_policy_t history_policy_arg, size_t depth_arg)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static QoSInitialization</type>
      <name>from_rmw</name>
      <anchorfile>structrclcpp_1_1QoSInitialization.html</anchorfile>
      <anchor>aa6954df3e252169e73d9776eec6edb5b</anchor>
      <arglist>(const rmw_qos_profile_t &amp;rmw_qos)</arglist>
    </member>
    <member kind="variable">
      <type>rmw_qos_history_policy_t</type>
      <name>history_policy</name>
      <anchorfile>structrclcpp_1_1QoSInitialization.html</anchorfile>
      <anchor>a17c878920d98f354578a2b0abce1aa3d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>size_t</type>
      <name>depth</name>
      <anchorfile>structrclcpp_1_1QoSInitialization.html</anchorfile>
      <anchor>af2be52303374c3c0bbe2df4ff3488d82</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::RateBase</name>
    <filename>classrclcpp_1_1RateBase.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~RateBase</name>
      <anchorfile>classrclcpp_1_1RateBase.html</anchorfile>
      <anchor>a9fb11c6718b71399b857b95c85ea7e06</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>sleep</name>
      <anchorfile>classrclcpp_1_1RateBase.html</anchorfile>
      <anchor>ad6444622103be41488eb58bf12bf45a9</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>is_steady</name>
      <anchorfile>classrclcpp_1_1RateBase.html</anchorfile>
      <anchor>aa8975f51922655656808a3624490a477</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>reset</name>
      <anchorfile>classrclcpp_1_1RateBase.html</anchorfile>
      <anchor>a3fbd56efff72fb3c5728e6d764b0494a</anchor>
      <arglist>()=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::RCLBadAlloc</name>
    <filename>classrclcpp_1_1exceptions_1_1RCLBadAlloc.html</filename>
    <base>rclcpp::exceptions::RCLErrorBase</base>
    <base>std::bad_alloc</base>
    <member kind="function">
      <type></type>
      <name>RCLBadAlloc</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLBadAlloc.html</anchorfile>
      <anchor>ac37be8d2891251729ec9bed6bd8e11e5</anchor>
      <arglist>(rcl_ret_t ret, const rcl_error_state_t *error_state)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>RCLBadAlloc</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLBadAlloc.html</anchorfile>
      <anchor>a9e95d39f41f129da8191fdb66bd7df7c</anchor>
      <arglist>(const RCLErrorBase &amp;base_exc)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::RCLError</name>
    <filename>classrclcpp_1_1exceptions_1_1RCLError.html</filename>
    <base>rclcpp::exceptions::RCLErrorBase</base>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>RCLError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLError.html</anchorfile>
      <anchor>a77767bc207e23b5b1044d452b1c5ee25</anchor>
      <arglist>(rcl_ret_t ret, const rcl_error_state_t *error_state, const std::string &amp;prefix)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>RCLError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLError.html</anchorfile>
      <anchor>a5bc034af0d0ae35ec5b410aaec394ad6</anchor>
      <arglist>(const RCLErrorBase &amp;base_exc, const std::string &amp;prefix)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::RCLErrorBase</name>
    <filename>classrclcpp_1_1exceptions_1_1RCLErrorBase.html</filename>
    <member kind="function">
      <type></type>
      <name>RCLErrorBase</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLErrorBase.html</anchorfile>
      <anchor>af658b49a4338d673f01595a8a26b32d2</anchor>
      <arglist>(rcl_ret_t ret, const rcl_error_state_t *error_state)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~RCLErrorBase</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLErrorBase.html</anchorfile>
      <anchor>a9382912529075e19ce6acdd1131b52cb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable">
      <type>rcl_ret_t</type>
      <name>ret</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLErrorBase.html</anchorfile>
      <anchor>a721a7bb718a14d00c881c553eaed50f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::string</type>
      <name>message</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLErrorBase.html</anchorfile>
      <anchor>a129210d2605c51cc3e3d01e01fe2e803</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::string</type>
      <name>file</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLErrorBase.html</anchorfile>
      <anchor>a2531db0261a39abfedb838223b784bf7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>size_t</type>
      <name>line</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLErrorBase.html</anchorfile>
      <anchor>a7d544b3597cfc9b75fc07f3d184fd85e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::string</type>
      <name>formatted_message</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLErrorBase.html</anchorfile>
      <anchor>a45cf80939589d78d3e51912f6d65fdd4</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::RCLInvalidArgument</name>
    <filename>classrclcpp_1_1exceptions_1_1RCLInvalidArgument.html</filename>
    <base>rclcpp::exceptions::RCLErrorBase</base>
    <base>std::invalid_argument</base>
    <member kind="function">
      <type></type>
      <name>RCLInvalidArgument</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLInvalidArgument.html</anchorfile>
      <anchor>a47022e8562670175fee58a6a943c0939</anchor>
      <arglist>(rcl_ret_t ret, const rcl_error_state_t *error_state, const std::string &amp;prefix)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>RCLInvalidArgument</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLInvalidArgument.html</anchorfile>
      <anchor>a0e90e17115686a792c4aea8cd1adf6fd</anchor>
      <arglist>(const RCLErrorBase &amp;base_exc, const std::string &amp;prefix)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::RCLInvalidROSArgsError</name>
    <filename>classrclcpp_1_1exceptions_1_1RCLInvalidROSArgsError.html</filename>
    <base>rclcpp::exceptions::RCLErrorBase</base>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>RCLInvalidROSArgsError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLInvalidROSArgsError.html</anchorfile>
      <anchor>a0e29cd4bc99ac6ebe080ab04cd5b9d39</anchor>
      <arglist>(rcl_ret_t ret, const rcl_error_state_t *error_state, const std::string &amp;prefix)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>RCLInvalidROSArgsError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1RCLInvalidROSArgsError.html</anchorfile>
      <anchor>a1f375dc4e886e4cd2582ed0ad6bddc91</anchor>
      <arglist>(const RCLErrorBase &amp;base_exc, const std::string &amp;prefix)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock::ReadMutex</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock_1_1ReadMutex.html</filename>
    <member kind="function">
      <type>void</type>
      <name>lock</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock_1_1ReadMutex.html</anchorfile>
      <anchor>af027651f5ac5e2e86979d92f6e0a6b6b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>unlock</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock_1_1ReadMutex.html</anchorfile>
      <anchor>a938d5fb41023607ca0db9c6b8e0bf86a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>ReadMutex</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock_1_1ReadMutex.html</anchorfile>
      <anchor>a9fb687553bc9a87995b0249191804d93</anchor>
      <arglist>(WritePreferringReadWriteLock &amp;parent_lock)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>WritePreferringReadWriteLock &amp;</type>
      <name>parent_lock_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock_1_1ReadMutex.html</anchorfile>
      <anchor>a6fb5ff7c12e6f5a37e176d74d6c58333</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>friend</type>
      <name>WritePreferringReadWriteLock</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock_1_1ReadMutex.html</anchorfile>
      <anchor>a1744fb525882e3b1a42e2d329b95a349</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::experimental::buffers::RingBufferImplementation</name>
    <filename>classrclcpp_1_1experimental_1_1buffers_1_1RingBufferImplementation.html</filename>
    <templarg></templarg>
    <base>rclcpp::experimental::buffers::BufferImplementationBase</base>
    <member kind="function">
      <type></type>
      <name>RingBufferImplementation</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1RingBufferImplementation.html</anchorfile>
      <anchor>a147fe8f2a7e10d4ce3522430a0e67735</anchor>
      <arglist>(size_t capacity)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~RingBufferImplementation</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1RingBufferImplementation.html</anchorfile>
      <anchor>a749d5a31d5bbe7a3839b4af7d4ded8b2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>enqueue</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1RingBufferImplementation.html</anchorfile>
      <anchor>afec5cd912847342bb7a43e1261c8d15d</anchor>
      <arglist>(BufferT request)</arglist>
    </member>
    <member kind="function">
      <type>BufferT</type>
      <name>dequeue</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1RingBufferImplementation.html</anchorfile>
      <anchor>a722550274d2d02b45e3cabfd897167bb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>next</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1RingBufferImplementation.html</anchorfile>
      <anchor>ae04085aa50f28600d4449e6ff7d71d0e</anchor>
      <arglist>(size_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>has_data</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1RingBufferImplementation.html</anchorfile>
      <anchor>a264650193286fb4715d9098771a004b9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_full</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1RingBufferImplementation.html</anchorfile>
      <anchor>a4c87c337277747f6290bbfc32df38604</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>clear</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1RingBufferImplementation.html</anchorfile>
      <anchor>a4391f447830072f5a3b9bf02c885f969</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::detail::RMWImplementationSpecificPayload</name>
    <filename>classrclcpp_1_1detail_1_1RMWImplementationSpecificPayload.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~RMWImplementationSpecificPayload</name>
      <anchorfile>classrclcpp_1_1detail_1_1RMWImplementationSpecificPayload.html</anchorfile>
      <anchor>aa99dff85a0979c3aee533382a989655c</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>has_been_customized</name>
      <anchorfile>classrclcpp_1_1detail_1_1RMWImplementationSpecificPayload.html</anchorfile>
      <anchor>a736ef49fcee0ba3a408fff6f4f6c4f31</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual const char *</type>
      <name>get_implementation_identifier</name>
      <anchorfile>classrclcpp_1_1detail_1_1RMWImplementationSpecificPayload.html</anchorfile>
      <anchor>a82e49a7fb64918d62ef9624015987fd5</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::detail::RMWImplementationSpecificPublisherPayload</name>
    <filename>classrclcpp_1_1detail_1_1RMWImplementationSpecificPublisherPayload.html</filename>
    <base>rclcpp::detail::RMWImplementationSpecificPayload</base>
    <member kind="function">
      <type></type>
      <name>~RMWImplementationSpecificPublisherPayload</name>
      <anchorfile>classrclcpp_1_1detail_1_1RMWImplementationSpecificPublisherPayload.html</anchorfile>
      <anchor>a3529d2ce7a5a2c338c5f0e838218ac5b</anchor>
      <arglist>() override=default</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>modify_rmw_publisher_options</name>
      <anchorfile>classrclcpp_1_1detail_1_1RMWImplementationSpecificPublisherPayload.html</anchorfile>
      <anchor>a2fc82aaf7f67e707d86f2132a01d9794</anchor>
      <arglist>(rmw_publisher_options_t &amp;rmw_publisher_options) const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::detail::RMWImplementationSpecificSubscriptionPayload</name>
    <filename>classrclcpp_1_1detail_1_1RMWImplementationSpecificSubscriptionPayload.html</filename>
    <base>rclcpp::detail::RMWImplementationSpecificPayload</base>
    <member kind="function">
      <type></type>
      <name>~RMWImplementationSpecificSubscriptionPayload</name>
      <anchorfile>classrclcpp_1_1detail_1_1RMWImplementationSpecificSubscriptionPayload.html</anchorfile>
      <anchor>acef5f3c89b2b66559e057d4ca701ba8e</anchor>
      <arglist>() override=default</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>modify_rmw_subscription_options</name>
      <anchorfile>classrclcpp_1_1detail_1_1RMWImplementationSpecificSubscriptionPayload.html</anchorfile>
      <anchor>a0bea397d9030d2f058b3eea5058cb7fb</anchor>
      <arglist>(rmw_subscription_options_t &amp;rmw_subscription_options) const</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::function_traits::same_arguments</name>
    <filename>structrclcpp_1_1function__traits_1_1same__arguments.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <base>is_same&lt; function_traits&lt; FunctorAT &gt;::arguments, function_traits&lt; FunctorBT &gt;::arguments &gt;</base>
  </compound>
  <compound kind="struct">
    <name>rclcpp::ScopeExit</name>
    <filename>structrclcpp_1_1ScopeExit.html</filename>
    <templarg></templarg>
    <member kind="function">
      <type></type>
      <name>ScopeExit</name>
      <anchorfile>structrclcpp_1_1ScopeExit.html</anchorfile>
      <anchor>a75486825f8f8504f6ef85cd5f8e8cda3</anchor>
      <arglist>(Callable callable)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ScopeExit</name>
      <anchorfile>structrclcpp_1_1ScopeExit.html</anchorfile>
      <anchor>a325a3d5a31c88c0ef85118335b836a88</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::SensorDataQoS</name>
    <filename>classrclcpp_1_1SensorDataQoS.html</filename>
    <base>rclcpp::QoS</base>
    <member kind="function">
      <type></type>
      <name>SensorDataQoS</name>
      <anchorfile>classrclcpp_1_1SensorDataQoS.html</anchorfile>
      <anchor>a36af0e759c7e12c29c09a6c6cf62c25f</anchor>
      <arglist>(const QoSInitialization &amp;qos_initialization=(QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)))</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::SequentialSynchronization</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</filename>
    <base>rclcpp::wait_set_policies::detail::SynchronizationPolicyCommon</base>
    <member kind="function" protection="protected">
      <type></type>
      <name>SequentialSynchronization</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a473d2648bfd159048fe3d91c31d8737a</anchor>
      <arglist>(rclcpp::Context::SharedPtr)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>~SequentialSynchronization</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a9dd5f8291c11c71f3eebb7af30c5d116</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>const std::array&lt; std::shared_ptr&lt; rclcpp::GuardCondition &gt;, 0 &gt; &amp;</type>
      <name>get_extra_guard_conditions</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>aaacbc33ffb25f53be07f8a0feb30d79d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_add_subscription</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a6865358d02e17dae4d99cdc931eabd48</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; &amp;&amp;subscription, const rclcpp::SubscriptionWaitSetMask &amp;mask, std::function&lt; void(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; &amp;&amp;, const rclcpp::SubscriptionWaitSetMask &amp;) &gt; add_subscription_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_remove_subscription</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a9a9a0b87264449a1ca948cbf939ebc92</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; &amp;&amp;subscription, const rclcpp::SubscriptionWaitSetMask &amp;mask, std::function&lt; void(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; &amp;&amp;, const rclcpp::SubscriptionWaitSetMask &amp;) &gt; remove_subscription_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_add_guard_condition</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>acd85fb58a039fcda95d952c485365c4f</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::GuardCondition &gt; &amp;&amp;guard_condition, std::function&lt; void(std::shared_ptr&lt; rclcpp::GuardCondition &gt; &amp;&amp;)&gt; add_guard_condition_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_remove_guard_condition</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>afb2a4108d7a8cb48aaede8ee005438ac</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::GuardCondition &gt; &amp;&amp;guard_condition, std::function&lt; void(std::shared_ptr&lt; rclcpp::GuardCondition &gt; &amp;&amp;)&gt; remove_guard_condition_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_add_timer</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a31cb07262761c5a17c636c2606ad8eca</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::TimerBase &gt; &amp;&amp;timer, std::function&lt; void(std::shared_ptr&lt; rclcpp::TimerBase &gt; &amp;&amp;)&gt; add_timer_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_remove_timer</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>af4997393344b73c66b56c9869032bf0f</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::TimerBase &gt; &amp;&amp;timer, std::function&lt; void(std::shared_ptr&lt; rclcpp::TimerBase &gt; &amp;&amp;)&gt; remove_timer_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_add_client</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>ab2bf6b3ccefb4f00f44affd68cbc46ad</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ClientBase &gt; &amp;&amp;client, std::function&lt; void(std::shared_ptr&lt; rclcpp::ClientBase &gt; &amp;&amp;)&gt; add_client_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_remove_client</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a396ce6f15008074b5f7f1b399b8045f7</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ClientBase &gt; &amp;&amp;client, std::function&lt; void(std::shared_ptr&lt; rclcpp::ClientBase &gt; &amp;&amp;)&gt; remove_client_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_add_service</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a123f1785dac58f7baef79ef014dbfcb2</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ServiceBase &gt; &amp;&amp;service, std::function&lt; void(std::shared_ptr&lt; rclcpp::ServiceBase &gt; &amp;&amp;)&gt; add_service_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_remove_service</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a1670bacdcbc1c3a630119d3df7aa789b</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ServiceBase &gt; &amp;&amp;service, std::function&lt; void(std::shared_ptr&lt; rclcpp::ServiceBase &gt; &amp;&amp;)&gt; remove_service_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_add_waitable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>aedf7c1304d42ca2703999ec0066a34ff</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Waitable &gt; &amp;&amp;waitable, std::shared_ptr&lt; void &gt; &amp;&amp;associated_entity, std::function&lt; void(std::shared_ptr&lt; rclcpp::Waitable &gt; &amp;&amp;, std::shared_ptr&lt; void &gt; &amp;&amp;) &gt; add_waitable_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_remove_waitable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a77713e03974e2e45568fab7de53d979d</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Waitable &gt; &amp;&amp;waitable, std::function&lt; void(std::shared_ptr&lt; rclcpp::Waitable &gt; &amp;&amp;)&gt; remove_waitable_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_prune_deleted_entities</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a49659b1ea289ef520127f4c8fa8604e9</anchor>
      <arglist>(std::function&lt; void()&gt; prune_deleted_entities_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>WaitResultT</type>
      <name>sync_wait</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a793530bfed89ff1a20014f4d41da443a</anchor>
      <arglist>(std::chrono::nanoseconds time_to_wait_ns, std::function&lt; void()&gt; rebuild_rcl_wait_set, std::function&lt; rcl_wait_set_t &amp;()&gt; get_rcl_wait_set, std::function&lt; WaitResultT(WaitResultKind wait_result_kind)&gt; create_wait_result)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_wait_result_acquire</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a0d3e9bb18dc70a5867fed756386f9ffd</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_wait_result_release</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1SequentialSynchronization.html</anchorfile>
      <anchor>a5dbc3c9ab50baa06f1983da1e2cda0ab</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Serialization</name>
    <filename>classrclcpp_1_1Serialization.html</filename>
    <templarg></templarg>
    <base>rclcpp::SerializationBase</base>
    <member kind="function">
      <type></type>
      <name>Serialization</name>
      <anchorfile>classrclcpp_1_1Serialization.html</anchorfile>
      <anchor>ae1b2a30dbd6120f219abc1c927176708</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::SerializationBase</name>
    <filename>classrclcpp_1_1SerializationBase.html</filename>
    <member kind="function">
      <type></type>
      <name>SerializationBase</name>
      <anchorfile>classrclcpp_1_1SerializationBase.html</anchorfile>
      <anchor>a25918443048e52aeda3f02d03fc79431</anchor>
      <arglist>(const rosidl_message_type_support_t *type_support)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~SerializationBase</name>
      <anchorfile>classrclcpp_1_1SerializationBase.html</anchorfile>
      <anchor>ab76f9d303e8fbf8321755324b8d4efac</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>serialize_message</name>
      <anchorfile>classrclcpp_1_1SerializationBase.html</anchorfile>
      <anchor>a7f13a06d7e7e69085b5640995dc361e8</anchor>
      <arglist>(const void *ros_message, SerializedMessage *serialized_message) const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>deserialize_message</name>
      <anchorfile>classrclcpp_1_1SerializationBase.html</anchorfile>
      <anchor>ad51944b30e45faab5876d24dd9d344c6</anchor>
      <arglist>(const SerializedMessage *serialized_message, void *ros_message) const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::SerializedMessage</name>
    <filename>classrclcpp_1_1SerializedMessage.html</filename>
    <member kind="function">
      <type></type>
      <name>SerializedMessage</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>ac883c45c0d266793cfcc062158794dc1</anchor>
      <arglist>(const rcl_allocator_t &amp;allocator=rcl_get_default_allocator())</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SerializedMessage</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>af9d22194b1cc1c216cd743c52d33dcc8</anchor>
      <arglist>(size_t initial_capacity, const rcl_allocator_t &amp;allocator=rcl_get_default_allocator())</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SerializedMessage</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>a73ec663e9af670f6e8c17b1b593f4c09</anchor>
      <arglist>(const SerializedMessage &amp;other)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SerializedMessage</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>a28f327d110c2e0c1697112bc2af9dcce</anchor>
      <arglist>(const rcl_serialized_message_t &amp;other)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SerializedMessage</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>abf355c35f279c98c51b854c7b32b9143</anchor>
      <arglist>(SerializedMessage &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SerializedMessage</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>ae2195e139aaf93f809a11fd8410060d9</anchor>
      <arglist>(rcl_serialized_message_t &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>SerializedMessage &amp;</type>
      <name>operator=</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>a73f2de364ae30c009283f05b55f1e6df</anchor>
      <arglist>(const SerializedMessage &amp;other)</arglist>
    </member>
    <member kind="function">
      <type>SerializedMessage &amp;</type>
      <name>operator=</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>a84efd0b409ea6c78528290d31f65f70e</anchor>
      <arglist>(const rcl_serialized_message_t &amp;other)</arglist>
    </member>
    <member kind="function">
      <type>SerializedMessage &amp;</type>
      <name>operator=</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>aaa155f4f477181bc9c9576c6b2a1fb76</anchor>
      <arglist>(SerializedMessage &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>SerializedMessage &amp;</type>
      <name>operator=</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>aefb29c8f80c0892bba51f7ca5d910636</anchor>
      <arglist>(rcl_serialized_message_t &amp;&amp;other)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~SerializedMessage</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>acd70188b1f646ac2a3041c17bbc18f9f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rcl_serialized_message_t &amp;</type>
      <name>get_rcl_serialized_message</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>a5844218af56b545b5edf5cd300ac16e2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rcl_serialized_message_t &amp;</type>
      <name>get_rcl_serialized_message</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>ae0c11c787fe7c2c0c6490956193f5579</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>size</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>a4ddc6cd88c85aace90e94a73afbd5e97</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>capacity</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>ae856d22f04fea187719c2e93eb902306</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>reserve</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>a5f47b18bfe10d7cd84752d4c2196e9c4</anchor>
      <arglist>(size_t capacity)</arglist>
    </member>
    <member kind="function">
      <type>rcl_serialized_message_t</type>
      <name>release_rcl_serialized_message</name>
      <anchorfile>classrclcpp_1_1SerializedMessage.html</anchorfile>
      <anchor>a0d96c38b32068e0fd06688f42d5bd3be</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Service</name>
    <filename>classrclcpp_1_1Service.html</filename>
    <templarg></templarg>
    <base>rclcpp::ServiceBase</base>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; typename ServiceT::Request &gt;, std::shared_ptr&lt; typename ServiceT::Response &gt;)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a8d9a188a189533c875f2e2c4f27c0a78</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; rmw_request_id_t &gt;, const std::shared_ptr&lt; typename ServiceT::Request &gt;, std::shared_ptr&lt; typename ServiceT::Response &gt;)&gt;</type>
      <name>CallbackWithHeaderType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>abd13c7b574f2a3b914e66afa21a397d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aefe6bc46d7b2cba2986ddd0b53f39cf7</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, const std::string &amp;service_name, AnyServiceCallback&lt; ServiceT &gt; any_callback, rcl_service_options_t &amp;service_options)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a74cef7444a758ae10d2bebd0a39e126c</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, std::shared_ptr&lt; rcl_service_t &gt; service_handle, AnyServiceCallback&lt; ServiceT &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>acf8b6cb404c808381987f2245b1a5c61</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, rcl_service_t *service_handle, AnyServiceCallback&lt; ServiceT &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a17dd92840daf41efea7ff89fb2a800d2</anchor>
      <arglist>()=delete</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a6e4615ce813944a5345a5b946459c969</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a16504ced8ebc7a570564a7c0a15914cd</anchor>
      <arglist>(typename ServiceT::Request &amp;request_out, rmw_request_id_t &amp;request_id_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a011e0e937d9a5addd1f90694e10246d6</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa0c6f38a6d5c8b58c6a7e9d598b0fa76</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a18498cd7f88695a3aa0ff38835f506c0</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; request) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa5d6c8b8831df0cd23b9ff08cd900826</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; req_id, std::shared_ptr&lt; typename ServiceT::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a855073d8d80581dc22daf2a21fb6253a</anchor>
      <arglist>(rmw_request_id_t &amp;req_id, typename ServiceT::Response &amp;response)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Service&lt; rcl_interfaces::srv::DescribeParameters &gt;</name>
    <filename>classrclcpp_1_1Service.html</filename>
    <base>rclcpp::ServiceBase</base>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; typename rcl_interfaces::srv::DescribeParameters ::Request &gt;, std::shared_ptr&lt; typename rcl_interfaces::srv::DescribeParameters ::Response &gt;)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a8d9a188a189533c875f2e2c4f27c0a78</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; rmw_request_id_t &gt;, const std::shared_ptr&lt; typename rcl_interfaces::srv::DescribeParameters ::Request &gt;, std::shared_ptr&lt; typename rcl_interfaces::srv::DescribeParameters ::Response &gt;)&gt;</type>
      <name>CallbackWithHeaderType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>abd13c7b574f2a3b914e66afa21a397d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aefe6bc46d7b2cba2986ddd0b53f39cf7</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, const std::string &amp;service_name, AnyServiceCallback&lt; rcl_interfaces::srv::DescribeParameters &gt; any_callback, rcl_service_options_t &amp;service_options)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a74cef7444a758ae10d2bebd0a39e126c</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, std::shared_ptr&lt; rcl_service_t &gt; service_handle, AnyServiceCallback&lt; rcl_interfaces::srv::DescribeParameters &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>acf8b6cb404c808381987f2245b1a5c61</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, rcl_service_t *service_handle, AnyServiceCallback&lt; rcl_interfaces::srv::DescribeParameters &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a17dd92840daf41efea7ff89fb2a800d2</anchor>
      <arglist>()=delete</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a6e4615ce813944a5345a5b946459c969</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a16504ced8ebc7a570564a7c0a15914cd</anchor>
      <arglist>(typename rcl_interfaces::srv::DescribeParameters ::Request &amp;request_out, rmw_request_id_t &amp;request_id_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a011e0e937d9a5addd1f90694e10246d6</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa0c6f38a6d5c8b58c6a7e9d598b0fa76</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a18498cd7f88695a3aa0ff38835f506c0</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; request) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa5d6c8b8831df0cd23b9ff08cd900826</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; req_id, std::shared_ptr&lt; typename rcl_interfaces::srv::DescribeParameters ::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a855073d8d80581dc22daf2a21fb6253a</anchor>
      <arglist>(rmw_request_id_t &amp;req_id, typename rcl_interfaces::srv::DescribeParameters ::Response &amp;response)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Service&lt; rcl_interfaces::srv::GetParameters &gt;</name>
    <filename>classrclcpp_1_1Service.html</filename>
    <base>rclcpp::ServiceBase</base>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameters ::Request &gt;, std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameters ::Response &gt;)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a8d9a188a189533c875f2e2c4f27c0a78</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; rmw_request_id_t &gt;, const std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameters ::Request &gt;, std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameters ::Response &gt;)&gt;</type>
      <name>CallbackWithHeaderType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>abd13c7b574f2a3b914e66afa21a397d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aefe6bc46d7b2cba2986ddd0b53f39cf7</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, const std::string &amp;service_name, AnyServiceCallback&lt; rcl_interfaces::srv::GetParameters &gt; any_callback, rcl_service_options_t &amp;service_options)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a74cef7444a758ae10d2bebd0a39e126c</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, std::shared_ptr&lt; rcl_service_t &gt; service_handle, AnyServiceCallback&lt; rcl_interfaces::srv::GetParameters &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>acf8b6cb404c808381987f2245b1a5c61</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, rcl_service_t *service_handle, AnyServiceCallback&lt; rcl_interfaces::srv::GetParameters &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a17dd92840daf41efea7ff89fb2a800d2</anchor>
      <arglist>()=delete</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a6e4615ce813944a5345a5b946459c969</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a16504ced8ebc7a570564a7c0a15914cd</anchor>
      <arglist>(typename rcl_interfaces::srv::GetParameters ::Request &amp;request_out, rmw_request_id_t &amp;request_id_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a011e0e937d9a5addd1f90694e10246d6</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa0c6f38a6d5c8b58c6a7e9d598b0fa76</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a18498cd7f88695a3aa0ff38835f506c0</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; request) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa5d6c8b8831df0cd23b9ff08cd900826</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; req_id, std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameters ::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a855073d8d80581dc22daf2a21fb6253a</anchor>
      <arglist>(rmw_request_id_t &amp;req_id, typename rcl_interfaces::srv::GetParameters ::Response &amp;response)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Service&lt; rcl_interfaces::srv::GetParameterTypes &gt;</name>
    <filename>classrclcpp_1_1Service.html</filename>
    <base>rclcpp::ServiceBase</base>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameterTypes ::Request &gt;, std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameterTypes ::Response &gt;)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a8d9a188a189533c875f2e2c4f27c0a78</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; rmw_request_id_t &gt;, const std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameterTypes ::Request &gt;, std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameterTypes ::Response &gt;)&gt;</type>
      <name>CallbackWithHeaderType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>abd13c7b574f2a3b914e66afa21a397d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aefe6bc46d7b2cba2986ddd0b53f39cf7</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, const std::string &amp;service_name, AnyServiceCallback&lt; rcl_interfaces::srv::GetParameterTypes &gt; any_callback, rcl_service_options_t &amp;service_options)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a74cef7444a758ae10d2bebd0a39e126c</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, std::shared_ptr&lt; rcl_service_t &gt; service_handle, AnyServiceCallback&lt; rcl_interfaces::srv::GetParameterTypes &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>acf8b6cb404c808381987f2245b1a5c61</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, rcl_service_t *service_handle, AnyServiceCallback&lt; rcl_interfaces::srv::GetParameterTypes &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a17dd92840daf41efea7ff89fb2a800d2</anchor>
      <arglist>()=delete</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a6e4615ce813944a5345a5b946459c969</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a16504ced8ebc7a570564a7c0a15914cd</anchor>
      <arglist>(typename rcl_interfaces::srv::GetParameterTypes ::Request &amp;request_out, rmw_request_id_t &amp;request_id_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a011e0e937d9a5addd1f90694e10246d6</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa0c6f38a6d5c8b58c6a7e9d598b0fa76</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a18498cd7f88695a3aa0ff38835f506c0</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; request) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa5d6c8b8831df0cd23b9ff08cd900826</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; req_id, std::shared_ptr&lt; typename rcl_interfaces::srv::GetParameterTypes ::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a855073d8d80581dc22daf2a21fb6253a</anchor>
      <arglist>(rmw_request_id_t &amp;req_id, typename rcl_interfaces::srv::GetParameterTypes ::Response &amp;response)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Service&lt; rcl_interfaces::srv::ListParameters &gt;</name>
    <filename>classrclcpp_1_1Service.html</filename>
    <base>rclcpp::ServiceBase</base>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; typename rcl_interfaces::srv::ListParameters ::Request &gt;, std::shared_ptr&lt; typename rcl_interfaces::srv::ListParameters ::Response &gt;)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a8d9a188a189533c875f2e2c4f27c0a78</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; rmw_request_id_t &gt;, const std::shared_ptr&lt; typename rcl_interfaces::srv::ListParameters ::Request &gt;, std::shared_ptr&lt; typename rcl_interfaces::srv::ListParameters ::Response &gt;)&gt;</type>
      <name>CallbackWithHeaderType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>abd13c7b574f2a3b914e66afa21a397d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aefe6bc46d7b2cba2986ddd0b53f39cf7</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, const std::string &amp;service_name, AnyServiceCallback&lt; rcl_interfaces::srv::ListParameters &gt; any_callback, rcl_service_options_t &amp;service_options)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a74cef7444a758ae10d2bebd0a39e126c</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, std::shared_ptr&lt; rcl_service_t &gt; service_handle, AnyServiceCallback&lt; rcl_interfaces::srv::ListParameters &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>acf8b6cb404c808381987f2245b1a5c61</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, rcl_service_t *service_handle, AnyServiceCallback&lt; rcl_interfaces::srv::ListParameters &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a17dd92840daf41efea7ff89fb2a800d2</anchor>
      <arglist>()=delete</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a6e4615ce813944a5345a5b946459c969</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a16504ced8ebc7a570564a7c0a15914cd</anchor>
      <arglist>(typename rcl_interfaces::srv::ListParameters ::Request &amp;request_out, rmw_request_id_t &amp;request_id_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a011e0e937d9a5addd1f90694e10246d6</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa0c6f38a6d5c8b58c6a7e9d598b0fa76</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a18498cd7f88695a3aa0ff38835f506c0</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; request) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa5d6c8b8831df0cd23b9ff08cd900826</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; req_id, std::shared_ptr&lt; typename rcl_interfaces::srv::ListParameters ::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a855073d8d80581dc22daf2a21fb6253a</anchor>
      <arglist>(rmw_request_id_t &amp;req_id, typename rcl_interfaces::srv::ListParameters ::Response &amp;response)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Service&lt; rcl_interfaces::srv::SetParameters &gt;</name>
    <filename>classrclcpp_1_1Service.html</filename>
    <base>rclcpp::ServiceBase</base>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; typename rcl_interfaces::srv::SetParameters ::Request &gt;, std::shared_ptr&lt; typename rcl_interfaces::srv::SetParameters ::Response &gt;)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a8d9a188a189533c875f2e2c4f27c0a78</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; rmw_request_id_t &gt;, const std::shared_ptr&lt; typename rcl_interfaces::srv::SetParameters ::Request &gt;, std::shared_ptr&lt; typename rcl_interfaces::srv::SetParameters ::Response &gt;)&gt;</type>
      <name>CallbackWithHeaderType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>abd13c7b574f2a3b914e66afa21a397d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aefe6bc46d7b2cba2986ddd0b53f39cf7</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, const std::string &amp;service_name, AnyServiceCallback&lt; rcl_interfaces::srv::SetParameters &gt; any_callback, rcl_service_options_t &amp;service_options)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a74cef7444a758ae10d2bebd0a39e126c</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, std::shared_ptr&lt; rcl_service_t &gt; service_handle, AnyServiceCallback&lt; rcl_interfaces::srv::SetParameters &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>acf8b6cb404c808381987f2245b1a5c61</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, rcl_service_t *service_handle, AnyServiceCallback&lt; rcl_interfaces::srv::SetParameters &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a17dd92840daf41efea7ff89fb2a800d2</anchor>
      <arglist>()=delete</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a6e4615ce813944a5345a5b946459c969</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a16504ced8ebc7a570564a7c0a15914cd</anchor>
      <arglist>(typename rcl_interfaces::srv::SetParameters ::Request &amp;request_out, rmw_request_id_t &amp;request_id_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a011e0e937d9a5addd1f90694e10246d6</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa0c6f38a6d5c8b58c6a7e9d598b0fa76</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a18498cd7f88695a3aa0ff38835f506c0</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; request) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa5d6c8b8831df0cd23b9ff08cd900826</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; req_id, std::shared_ptr&lt; typename rcl_interfaces::srv::SetParameters ::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a855073d8d80581dc22daf2a21fb6253a</anchor>
      <arglist>(rmw_request_id_t &amp;req_id, typename rcl_interfaces::srv::SetParameters ::Response &amp;response)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Service&lt; rcl_interfaces::srv::SetParametersAtomically &gt;</name>
    <filename>classrclcpp_1_1Service.html</filename>
    <base>rclcpp::ServiceBase</base>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; typename rcl_interfaces::srv::SetParametersAtomically ::Request &gt;, std::shared_ptr&lt; typename rcl_interfaces::srv::SetParametersAtomically ::Response &gt;)&gt;</type>
      <name>CallbackType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a8d9a188a189533c875f2e2c4f27c0a78</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(const std::shared_ptr&lt; rmw_request_id_t &gt;, const std::shared_ptr&lt; typename rcl_interfaces::srv::SetParametersAtomically ::Request &gt;, std::shared_ptr&lt; typename rcl_interfaces::srv::SetParametersAtomically ::Response &gt;)&gt;</type>
      <name>CallbackWithHeaderType</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>abd13c7b574f2a3b914e66afa21a397d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aefe6bc46d7b2cba2986ddd0b53f39cf7</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, const std::string &amp;service_name, AnyServiceCallback&lt; rcl_interfaces::srv::SetParametersAtomically &gt; any_callback, rcl_service_options_t &amp;service_options)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a74cef7444a758ae10d2bebd0a39e126c</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, std::shared_ptr&lt; rcl_service_t &gt; service_handle, AnyServiceCallback&lt; rcl_interfaces::srv::SetParametersAtomically &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>acf8b6cb404c808381987f2245b1a5c61</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle, rcl_service_t *service_handle, AnyServiceCallback&lt; rcl_interfaces::srv::SetParametersAtomically &gt; any_callback)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a17dd92840daf41efea7ff89fb2a800d2</anchor>
      <arglist>()=delete</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Service</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a6e4615ce813944a5345a5b946459c969</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a16504ced8ebc7a570564a7c0a15914cd</anchor>
      <arglist>(typename rcl_interfaces::srv::SetParametersAtomically ::Request &amp;request_out, rmw_request_id_t &amp;request_id_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a011e0e937d9a5addd1f90694e10246d6</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa0c6f38a6d5c8b58c6a7e9d598b0fa76</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_request</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a18498cd7f88695a3aa0ff38835f506c0</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; request) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>aa5d6c8b8831df0cd23b9ff08cd900826</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; req_id, std::shared_ptr&lt; typename rcl_interfaces::srv::SetParametersAtomically ::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>send_response</name>
      <anchorfile>classrclcpp_1_1Service.html</anchorfile>
      <anchor>a855073d8d80581dc22daf2a21fb6253a</anchor>
      <arglist>(rmw_request_id_t &amp;req_id, typename rcl_interfaces::srv::SetParametersAtomically ::Response &amp;response)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::ServiceBase</name>
    <filename>classrclcpp_1_1ServiceBase.html</filename>
    <member kind="function">
      <type></type>
      <name>ServiceBase</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>ac06b1dd4c32c6c8df760ee11fddf56a1</anchor>
      <arglist>(std::shared_ptr&lt; rcl_node_t &gt; node_handle)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ServiceBase</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>a054d411b93922381af61ef3de563f502</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_service_name</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>a1ed32cc7be0eb8ee1bef728d6fa2505f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rcl_service_t &gt;</type>
      <name>get_service_handle</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>aca87a073f581b000bee13d815574afb4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; const rcl_service_t &gt;</type>
      <name>get_service_handle</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>ad7eaec1590907454d12e9ac4e840060b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_type_erased_request</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>aea9f18b8a10baddf79e509b7cb6e4814</anchor>
      <arglist>(void *request_out, rmw_request_id_t &amp;request_id_out)</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::shared_ptr&lt; void &gt;</type>
      <name>create_request</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>a7ce14b74e192a4ca3b55594d228b7aa0</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::shared_ptr&lt; rmw_request_id_t &gt;</type>
      <name>create_request_header</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>a8a4eb2a61c9232f4089111bfa18147bd</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>handle_request</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>a5f97cda8d96e346edd8d6c08d4a31103</anchor>
      <arglist>(std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; void &gt; request)=0</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>exchange_in_use_by_wait_set_state</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>a348fc38de259c53f35184d20f33e5fb5</anchor>
      <arglist>(bool in_use_state)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>rcl_node_t *</type>
      <name>get_rcl_node_handle</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>acec7adcb1948eabb8eb28559cccd81f8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>const rcl_node_t *</type>
      <name>get_rcl_node_handle</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>a47ee6eb3e32e0f2ef66c43d77aad7786</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rcl_node_t &gt;</type>
      <name>node_handle_</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>aac297e598e51dbecc04bb713639201e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rcl_service_t &gt;</type>
      <name>service_handle_</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>abbeb4d8962d3e9ba453540be237d13f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>bool</type>
      <name>owns_rcl_handle_</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>a4d5c4c684735e75428e03d4691b35dd9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::atomic&lt; bool &gt;</type>
      <name>in_use_by_wait_set_</name>
      <anchorfile>classrclcpp_1_1ServiceBase.html</anchorfile>
      <anchor>ace8c4bee0c5273e1d2efaf6514c93f2a</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::ServicesQoS</name>
    <filename>classrclcpp_1_1ServicesQoS.html</filename>
    <base>rclcpp::QoS</base>
    <member kind="function">
      <type></type>
      <name>ServicesQoS</name>
      <anchorfile>classrclcpp_1_1ServicesQoS.html</anchorfile>
      <anchor>a33e222b46c44364a6b6e973b0db176a8</anchor>
      <arglist>(const QoSInitialization &amp;qos_initialization=(QoSInitialization::from_rmw(rmw_qos_profile_services_default)))</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::executors::SingleThreadedExecutor</name>
    <filename>classrclcpp_1_1executors_1_1SingleThreadedExecutor.html</filename>
    <base>rclcpp::Executor</base>
    <member kind="function">
      <type></type>
      <name>SingleThreadedExecutor</name>
      <anchorfile>classrclcpp_1_1executors_1_1SingleThreadedExecutor.html</anchorfile>
      <anchor>abe7a460308894425f11cf19ddc4ac6e4</anchor>
      <arglist>(const rclcpp::ExecutorOptions &amp;options=rclcpp::ExecutorOptions())</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~SingleThreadedExecutor</name>
      <anchorfile>classrclcpp_1_1executors_1_1SingleThreadedExecutor.html</anchorfile>
      <anchor>a1a9571d94eca4a614ae0a0335cdc4c8e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin</name>
      <anchorfile>classrclcpp_1_1executors_1_1SingleThreadedExecutor.html</anchorfile>
      <anchor>a77218fef5f77a2084a251e7b885fd3d0</anchor>
      <arglist>() override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::executors::StaticExecutorEntitiesCollector</name>
    <filename>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</filename>
    <base>rclcpp::Waitable</base>
    <base>enable_shared_from_this&lt; StaticExecutorEntitiesCollector &gt;</base>
    <member kind="function">
      <type></type>
      <name>StaticExecutorEntitiesCollector</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>ac5907ca9f2c04a8b89785eb0870cb66f</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~StaticExecutorEntitiesCollector</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a0a43e85a475047bc7fda4fc8f2fb8a60</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>init</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>ab96683d3b73c6026e4782d34747ef4bd</anchor>
      <arglist>(rcl_wait_set_t *p_wait_set, rclcpp::memory_strategy::MemoryStrategy::SharedPtr &amp;memory_strategy, rcl_guard_condition_t *executor_guard_condition)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>fini</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a0f5e9ca44c00ac704d8c96f9753cf87d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>execute</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a3633bfd06578902a7978a8758d061cce</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>fill_memory_strategy</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>ae499a613cac5a090a2fd56cfc56c6866</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>fill_executable_list</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>ac6f58388986866a0df8a268c1256fb2c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>prepare_wait_set</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a55f67af2a9015495f7814ca212facfb8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>refresh_wait_set</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a965468c8ccb38788f4a5e5c35d77ee28</anchor>
      <arglist>(std::chrono::nanoseconds timeout=std::chrono::nanoseconds(-1))</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>add_to_wait_set</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a1ca55d0728cd4fb1e63ba70b56dfc336</anchor>
      <arglist>(rcl_wait_set_t *wait_set) override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_number_of_ready_guard_conditions</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a9e9bdb0ad84b84a2b10798ebd92d178c</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_node</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>af66a950cf115f29bb1e9fa295b8ee5d9</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>remove_node</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>af8fbdcfe915177d655ae4f87062b772e</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_ready</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a5bc78d0279b71cdf42831c3ac003c28d</anchor>
      <arglist>(rcl_wait_set_t *wait_set) override</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_number_of_timers</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a677bc1334fc5ceffcba92a3f9deffefb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_number_of_subscriptions</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a4ef0717a1ddec3007900fe79a28be6ab</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_number_of_services</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a38d5992859169a73edb3a0b3753a21eb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_number_of_clients</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a16fce738efc898f6f481ce22177ced47</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_number_of_waitables</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a2c06aca3cc8cf629cf4b33544fc920ff</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::SubscriptionBase::SharedPtr</type>
      <name>get_subscription</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a6cc2c8691e43a19c787ad3c617ff9548</anchor>
      <arglist>(size_t i)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::TimerBase::SharedPtr</type>
      <name>get_timer</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a6400469ddec3abc9f224c7af72dabcb1</anchor>
      <arglist>(size_t i)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::ServiceBase::SharedPtr</type>
      <name>get_service</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a70c3dcd0a2f512ccc908dfcbd37d1804</anchor>
      <arglist>(size_t i)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::ClientBase::SharedPtr</type>
      <name>get_client</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a9ed4ad0048c0e22ba60d68a9d045e2f0</anchor>
      <arglist>(size_t i)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Waitable::SharedPtr</type>
      <name>get_waitable</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticExecutorEntitiesCollector.html</anchorfile>
      <anchor>a3b110b732ad33f8af3bbd7f6dd2ab4ad</anchor>
      <arglist>(size_t i)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::executors::StaticSingleThreadedExecutor</name>
    <filename>classrclcpp_1_1executors_1_1StaticSingleThreadedExecutor.html</filename>
    <base>rclcpp::Executor</base>
    <member kind="function">
      <type></type>
      <name>StaticSingleThreadedExecutor</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticSingleThreadedExecutor.html</anchorfile>
      <anchor>a6ee69a543c46f731a74a8d33fe6bf3e5</anchor>
      <arglist>(const rclcpp::ExecutorOptions &amp;options=rclcpp::ExecutorOptions())</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~StaticSingleThreadedExecutor</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticSingleThreadedExecutor.html</anchorfile>
      <anchor>acfba5ffd5adf68e26137f49a8587ad9e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticSingleThreadedExecutor.html</anchorfile>
      <anchor>a1cec9027c4c69e49a6e15398f01b7c47</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_node</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticSingleThreadedExecutor.html</anchorfile>
      <anchor>a5c04102c89554658c021dceb1555af2f</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify=true) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_node</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticSingleThreadedExecutor.html</anchorfile>
      <anchor>acc40d425a0b14000dcec22b93a5ba194</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; node_ptr, bool notify=true) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_node</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticSingleThreadedExecutor.html</anchorfile>
      <anchor>ad0c159ea2f81661d52b06b1e8a4edbd0</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify=true) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_node</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticSingleThreadedExecutor.html</anchorfile>
      <anchor>a9578b4ab98dda3b6493d35b87c302af4</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; node_ptr, bool notify=true) override</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::FutureReturnCode</type>
      <name>spin_until_future_complete</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticSingleThreadedExecutor.html</anchorfile>
      <anchor>ab8aed736c98d24c72bd26db690d6e3ba</anchor>
      <arglist>(std::shared_future&lt; ResponseT &gt; &amp;future, std::chrono::duration&lt; TimeRepT, TimeT &gt; timeout=std::chrono::duration&lt; TimeRepT, TimeT &gt;(-1))</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>execute_ready_executables</name>
      <anchorfile>classrclcpp_1_1executors_1_1StaticSingleThreadedExecutor.html</anchorfile>
      <anchor>ab767fee73876b7a64cdc98ce9de0a9aa</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::StaticStorage</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</filename>
    <templarg>NumberOfSubscriptions</templarg>
    <templarg>NumberOfGuardCondtions</templarg>
    <templarg>NumberOfTimers</templarg>
    <templarg>NumberOfClients</templarg>
    <templarg>NumberOfServices</templarg>
    <templarg>NumberOfWaitables</templarg>
    <base>StoragePolicyCommon&lt; true &gt;</base>
    <class kind="class">rclcpp::wait_set_policies::StaticStorage::SubscriptionEntry</class>
    <class kind="struct">rclcpp::wait_set_policies::StaticStorage::WaitableEntry</class>
    <member kind="typedef" protection="protected">
      <type>std::false_type</type>
      <name>is_mutable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>aa7b5e5e086bf4666255296954e6aa310</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::array&lt; SubscriptionEntry, NumberOfSubscriptions &gt;</type>
      <name>ArrayOfSubscriptions</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a1e8f7214b3e5ec99b65f2573a2de6ac4</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>ArrayOfSubscriptions</type>
      <name>SubscriptionsIterable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a24a9ac1fe6cb319cc7cb40c5379b41d9</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::array&lt; std::shared_ptr&lt; rclcpp::GuardCondition &gt;, NumberOfGuardCondtions &gt;</type>
      <name>ArrayOfGuardConditions</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>af3547f974f4686bfd40c9efabbb9e16b</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>ArrayOfGuardConditions</type>
      <name>GuardConditionsIterable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a6be6fc9760c83d30c92a98816d6f19b5</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::array&lt; std::shared_ptr&lt; rclcpp::TimerBase &gt;, NumberOfTimers &gt;</type>
      <name>ArrayOfTimers</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a9c4fff74ceb5674847e18f4d98bfca2c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>ArrayOfTimers</type>
      <name>TimersIterable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>ab20f6d44ee522eca3ed3e2d8046fb059</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::array&lt; std::shared_ptr&lt; rclcpp::ClientBase &gt;, NumberOfClients &gt;</type>
      <name>ArrayOfClients</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>ab93bf93e242c3734d7c911073e4b1817</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>ArrayOfClients</type>
      <name>ClientsIterable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a1b30a39150384c0a62f598c3d5ddeaec</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::array&lt; std::shared_ptr&lt; rclcpp::ServiceBase &gt;, NumberOfServices &gt;</type>
      <name>ArrayOfServices</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a1e7f44e9958c32e3ed83522c27d5b398</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>ArrayOfServices</type>
      <name>ServicesIterable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a1cd471e1fcc85150312c7cb8096da9a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>std::array&lt; WaitableEntry, NumberOfWaitables &gt;</type>
      <name>ArrayOfWaitables</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>af801d9c137ed34fe84456ddcc44df5b5</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="protected">
      <type>ArrayOfWaitables</type>
      <name>WaitablesIterable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a011770a10b39d11b8d9e8ac329e1756c</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>StaticStorage</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>ad077723c6e8aa8295fc46b0752b2d106</anchor>
      <arglist>(const ArrayOfSubscriptions &amp;subscriptions, const ArrayOfGuardConditions &amp;guard_conditions, const ArrayOfExtraGuardConditions &amp;extra_guard_conditions, const ArrayOfTimers &amp;timers, const ArrayOfClients &amp;clients, const ArrayOfServices &amp;services, const ArrayOfWaitables &amp;waitables, rclcpp::Context::SharedPtr context)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>~StaticStorage</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a72913b79e0913a7eba3e7d8cde7c495a</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_rebuild_rcl_wait_set</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a83d19a21f365da8c3f81c69eb3ec319d</anchor>
      <arglist>(const ArrayOfExtraGuardConditions &amp;extra_guard_conditions)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_acquire_ownerships</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a52eb1da4492d24e02632760aaae47726</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_release_ownerships</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a9e4805c56d0b8fbae1c0e6f4600746b3</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const ArrayOfSubscriptions</type>
      <name>subscriptions_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>aeb532cd1ba410e588192072b1c52917b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const ArrayOfGuardConditions</type>
      <name>guard_conditions_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>ac244cce8675dcf5a023a8020c4f0e67b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const ArrayOfTimers</type>
      <name>timers_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>ad7a046a136b9fc22cae2bfd90d15a047</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const ArrayOfClients</type>
      <name>clients_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>af36acf5b102a8997455b45b3cbd4b6d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const ArrayOfServices</type>
      <name>services_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>abd4464860d0b5db7b8b0dbbe9a345b1a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>const ArrayOfWaitables</type>
      <name>waitables_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage.html</anchorfile>
      <anchor>a6f69a9f0ef4c8d8446b8a0f0b45e3816</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::detail::StoragePolicyCommon</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</filename>
    <templarg>HasStrongOwnership</templarg>
    <member kind="function" protection="protected">
      <type></type>
      <name>StoragePolicyCommon</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a56bb00a5fe25a63f5c6cded62ade8520</anchor>
      <arglist>(const SubscriptionsIterable &amp;subscriptions, const GuardConditionsIterable &amp;guard_conditions, const ExtraGuardConditionsIterable &amp;extra_guard_conditions, const TimersIterable &amp;timers, const ClientsIterable &amp;clients, const ServicesIterable &amp;services, const WaitablesIterable &amp;waitables, rclcpp::Context::SharedPtr context)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>~StoragePolicyCommon</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a1f79659fba824f6f4a44e402342895b4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::pair&lt; void *, EntityT * &gt;</type>
      <name>get_raw_pointer_from_smart_pointer</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>aaf1db3e71867260b59e6ef4f890b577d</anchor>
      <arglist>(const std::shared_ptr&lt; EntityT &gt; &amp;shared_pointer)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::pair&lt; std::shared_ptr&lt; EntityT &gt;, EntityT * &gt;</type>
      <name>get_raw_pointer_from_smart_pointer</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a6242f865f81f6ce0c6c2c85b69562155</anchor>
      <arglist>(const std::weak_ptr&lt; EntityT &gt; &amp;weak_pointer)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_rebuild_rcl_wait_set_with_sets</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>ae237c2b172f4950da3c9e4953b5c1ecb</anchor>
      <arglist>(const SubscriptionsIterable &amp;subscriptions, const GuardConditionsIterable &amp;guard_conditions, const ExtraGuardConditionsIterable &amp;extra_guard_conditions, const TimersIterable &amp;timers, const ClientsIterable &amp;clients, const ServicesIterable &amp;services, const WaitablesIterable &amp;waitables)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>const rcl_wait_set_t &amp;</type>
      <name>storage_get_rcl_wait_set</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a5d651f5becf107e1c5d484bdcf791562</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>rcl_wait_set_t &amp;</type>
      <name>storage_get_rcl_wait_set</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a7fd09852ca6f7613b4324a90883638fd</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_flag_for_resize</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>ac0eb6058ab696779c2ddde28ee8317f1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rcl_wait_set_t</type>
      <name>rcl_wait_set_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a9c0e32728a970c42df7f50c4c155df66</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Context::SharedPtr</type>
      <name>context_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>ab6b2fb1463fe704fd73597e351c8f9b5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>bool</type>
      <name>needs_pruning_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>af62971a1e5404f75b6283ed0c4531692</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>bool</type>
      <name>needs_resize_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a1b0b14d7d4c1731781158b943b099bc4</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>StoragePolicyCommon&lt; false &gt;</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</filename>
    <member kind="function" protection="protected">
      <type></type>
      <name>StoragePolicyCommon</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a56bb00a5fe25a63f5c6cded62ade8520</anchor>
      <arglist>(const SubscriptionsIterable &amp;subscriptions, const GuardConditionsIterable &amp;guard_conditions, const ExtraGuardConditionsIterable &amp;extra_guard_conditions, const TimersIterable &amp;timers, const ClientsIterable &amp;clients, const ServicesIterable &amp;services, const WaitablesIterable &amp;waitables, rclcpp::Context::SharedPtr context)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>~StoragePolicyCommon</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a1f79659fba824f6f4a44e402342895b4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::pair&lt; void *, EntityT * &gt;</type>
      <name>get_raw_pointer_from_smart_pointer</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>aaf1db3e71867260b59e6ef4f890b577d</anchor>
      <arglist>(const std::shared_ptr&lt; EntityT &gt; &amp;shared_pointer)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::pair&lt; std::shared_ptr&lt; EntityT &gt;, EntityT * &gt;</type>
      <name>get_raw_pointer_from_smart_pointer</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a6242f865f81f6ce0c6c2c85b69562155</anchor>
      <arglist>(const std::weak_ptr&lt; EntityT &gt; &amp;weak_pointer)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_rebuild_rcl_wait_set_with_sets</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>ae237c2b172f4950da3c9e4953b5c1ecb</anchor>
      <arglist>(const SubscriptionsIterable &amp;subscriptions, const GuardConditionsIterable &amp;guard_conditions, const ExtraGuardConditionsIterable &amp;extra_guard_conditions, const TimersIterable &amp;timers, const ClientsIterable &amp;clients, const ServicesIterable &amp;services, const WaitablesIterable &amp;waitables)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>const rcl_wait_set_t &amp;</type>
      <name>storage_get_rcl_wait_set</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a5d651f5becf107e1c5d484bdcf791562</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>rcl_wait_set_t &amp;</type>
      <name>storage_get_rcl_wait_set</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a7fd09852ca6f7613b4324a90883638fd</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_flag_for_resize</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>ac0eb6058ab696779c2ddde28ee8317f1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rcl_wait_set_t</type>
      <name>rcl_wait_set_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a9c0e32728a970c42df7f50c4c155df66</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Context::SharedPtr</type>
      <name>context_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>ab6b2fb1463fe704fd73597e351c8f9b5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>bool</type>
      <name>needs_pruning_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>af62971a1e5404f75b6283ed0c4531692</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>bool</type>
      <name>needs_resize_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a1b0b14d7d4c1731781158b943b099bc4</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>StoragePolicyCommon&lt; true &gt;</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</filename>
    <member kind="function" protection="protected">
      <type></type>
      <name>StoragePolicyCommon</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a56bb00a5fe25a63f5c6cded62ade8520</anchor>
      <arglist>(const SubscriptionsIterable &amp;subscriptions, const GuardConditionsIterable &amp;guard_conditions, const ExtraGuardConditionsIterable &amp;extra_guard_conditions, const TimersIterable &amp;timers, const ClientsIterable &amp;clients, const ServicesIterable &amp;services, const WaitablesIterable &amp;waitables, rclcpp::Context::SharedPtr context)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>~StoragePolicyCommon</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a1f79659fba824f6f4a44e402342895b4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::pair&lt; void *, EntityT * &gt;</type>
      <name>get_raw_pointer_from_smart_pointer</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>aaf1db3e71867260b59e6ef4f890b577d</anchor>
      <arglist>(const std::shared_ptr&lt; EntityT &gt; &amp;shared_pointer)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::pair&lt; std::shared_ptr&lt; EntityT &gt;, EntityT * &gt;</type>
      <name>get_raw_pointer_from_smart_pointer</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a6242f865f81f6ce0c6c2c85b69562155</anchor>
      <arglist>(const std::weak_ptr&lt; EntityT &gt; &amp;weak_pointer)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_rebuild_rcl_wait_set_with_sets</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>ae237c2b172f4950da3c9e4953b5c1ecb</anchor>
      <arglist>(const SubscriptionsIterable &amp;subscriptions, const GuardConditionsIterable &amp;guard_conditions, const ExtraGuardConditionsIterable &amp;extra_guard_conditions, const TimersIterable &amp;timers, const ClientsIterable &amp;clients, const ServicesIterable &amp;services, const WaitablesIterable &amp;waitables)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>const rcl_wait_set_t &amp;</type>
      <name>storage_get_rcl_wait_set</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a5d651f5becf107e1c5d484bdcf791562</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>rcl_wait_set_t &amp;</type>
      <name>storage_get_rcl_wait_set</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a7fd09852ca6f7613b4324a90883638fd</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>storage_flag_for_resize</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>ac0eb6058ab696779c2ddde28ee8317f1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rcl_wait_set_t</type>
      <name>rcl_wait_set_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a9c0e32728a970c42df7f50c4c155df66</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::Context::SharedPtr</type>
      <name>context_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>ab6b2fb1463fe704fd73597e351c8f9b5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>bool</type>
      <name>needs_pruning_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>af62971a1e5404f75b6283ed0c4531692</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>bool</type>
      <name>needs_resize_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1StoragePolicyCommon.html</anchorfile>
      <anchor>a1b0b14d7d4c1731781158b943b099bc4</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Subscription</name>
    <filename>classrclcpp_1_1Subscription.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <base>rclcpp::SubscriptionBase</base>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; CallbackMessageT, AllocatorT &gt;</type>
      <name>MessageAllocatorTraits</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a43d6f64ef7ea732e90223b59fbcdeaee</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename MessageAllocatorTraits::allocator_type</type>
      <name>MessageAllocator</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a62d758318e74744d66c053c2710bd019</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>allocator::Deleter&lt; MessageAllocator, CallbackMessageT &gt;</type>
      <name>MessageDeleter</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a5cee8dd375d82b5d26c882c5e3d1bda8</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; const CallbackMessageT &gt;</type>
      <name>ConstMessageSharedPtr</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a3b06ec8a0e4210ff2445986229e0c806</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::unique_ptr&lt; CallbackMessageT, MessageDeleter &gt;</type>
      <name>MessageUniquePtr</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a561905ce7a5694b266ab51f3d83752b0</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; rclcpp::topic_statistics::SubscriptionTopicStatistics&lt; CallbackMessageT &gt; &gt;</type>
      <name>SubscriptionTopicStatisticsSharedPtr</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a467627594bb92ca99f4606e840181710</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Subscription</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a237c0b7290a12031bca39bf44da2040b</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, const rosidl_message_type_support_t &amp;type_support_handle, const std::string &amp;topic_name, const rclcpp::QoS &amp;qos, AnySubscriptionCallback&lt; CallbackMessageT, AllocatorT &gt; callback, const rclcpp::SubscriptionOptionsWithAllocator&lt; AllocatorT &gt; &amp;options, typename MessageMemoryStrategyT::SharedPtr message_memory_strategy, SubscriptionTopicStatisticsSharedPtr subscription_topic_statistics=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>post_init_setup</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a4ab3394ca057562e925b8ed9f725ebb8</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, const rclcpp::QoS &amp;qos, const rclcpp::SubscriptionOptionsWithAllocator&lt; AllocatorT &gt; &amp;options)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a6456f238c718d14c3c4fb771f892d20d</anchor>
      <arglist>(CallbackMessageT &amp;message_out, rclcpp::MessageInfo &amp;message_info_out)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>create_message</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>afd775823ea1272f3bbe3033e550929ac</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rclcpp::SerializedMessage &gt;</type>
      <name>create_serialized_message</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>aa65fb4815dafcac47e7484d9fe4c99bc</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_message</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a2368422a81363874a8fbfb4b2c01cbfa</anchor>
      <arglist>(std::shared_ptr&lt; void &gt; &amp;message, const rclcpp::MessageInfo &amp;message_info) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handle_loaned_message</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a23484777d5f0b70182c694824fe4a6a6</anchor>
      <arglist>(void *loaned_message, const rclcpp::MessageInfo &amp;message_info) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>return_message</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a3e8e7f6f3b6262d801ac0cb3a2f85571</anchor>
      <arglist>(std::shared_ptr&lt; void &gt; &amp;message) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>return_serialized_message</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>aeeffe9058298142681c7fed38b263139</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SerializedMessage &gt; &amp;message) override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>use_take_shared_method</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a65a3accbc4b0e7a35e275f5499e9656b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="friend">
      <type>friend class</type>
      <name>rclcpp::node_interfaces::NodeTopicsInterface</name>
      <anchorfile>classrclcpp_1_1Subscription.html</anchorfile>
      <anchor>a872235940e232040a0e05d383f0d262a</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::SubscriptionBase</name>
    <filename>classrclcpp_1_1SubscriptionBase.html</filename>
    <base>enable_shared_from_this&lt; SubscriptionBase &gt;</base>
    <member kind="typedef">
      <type>std::weak_ptr&lt; rclcpp::experimental::IntraProcessManager &gt;</type>
      <name>IntraProcessManagerWeakPtr</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a5acbc58580037d661aa735fddad70ba9</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SubscriptionBase</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a9f68a3c9cf39f5d8ddd0e318e23a0ada</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface *node_base, const rosidl_message_type_support_t &amp;type_support_handle, const std::string &amp;topic_name, const rcl_subscription_options_t &amp;subscription_options, bool is_serialized=false)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~SubscriptionBase</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a52b8cda0f87d9c1200b32800a39c0363</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_topic_name</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a71b149cae2bcec14c99ee32828ef7d8c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rcl_subscription_t &gt;</type>
      <name>get_subscription_handle</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a819a46b4c3ffba9bef8d05ad7eab7344</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; const rcl_subscription_t &gt;</type>
      <name>get_subscription_handle</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>ab065197aa67d19cf4eba0688ffedfccf</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; std::shared_ptr&lt; rclcpp::QOSEventHandlerBase &gt; &gt; &amp;</type>
      <name>get_event_handlers</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a87f5c2eddcc923bf981391a8b5a05e5b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::QoS</type>
      <name>get_actual_qos</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>abdaeff8614370f5e8b896f815928601a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_type_erased</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>aaee028d0dc23bcd2c0bfded09b5012cb</anchor>
      <arglist>(void *message_out, rclcpp::MessageInfo &amp;message_info_out)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>take_serialized</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a25ac489152fd7aa9ca31610cf6fb14dd</anchor>
      <arglist>(rclcpp::SerializedMessage &amp;message_out, rclcpp::MessageInfo &amp;message_info_out)</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::shared_ptr&lt; void &gt;</type>
      <name>create_message</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a82e353f7653570b836051428f02a2171</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual std::shared_ptr&lt; rclcpp::SerializedMessage &gt;</type>
      <name>create_serialized_message</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>ab44968e2dc8ae425dfc85c1882f67df7</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>handle_message</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a9e9859bf785042e896c8a9962ead8f89</anchor>
      <arglist>(std::shared_ptr&lt; void &gt; &amp;message, const rclcpp::MessageInfo &amp;message_info)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>handle_loaned_message</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>af6b7009c6a6b7f2363009f154243c86e</anchor>
      <arglist>(void *loaned_message, const rclcpp::MessageInfo &amp;message_info)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>return_message</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>ac44162910e6e0c184eaa1bf729287c55</anchor>
      <arglist>(std::shared_ptr&lt; void &gt; &amp;message)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>return_serialized_message</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a9b6b7186400304c5a7f4fd97198442a6</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SerializedMessage &gt; &amp;message)=0</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_message_type_support_t &amp;</type>
      <name>get_message_type_support_handle</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>aa1640e5f85ee0117f55f69b435723070</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_serialized</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a03df6a21a75e1eb4619a31b1b0ef852a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_publisher_count</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a0f61104666e697839925e0edc060359a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>can_loan_messages</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>aec59d8a5befb5d79c33b8701c86302bf</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setup_intra_process</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a1d01277c7ddfe0c33074154d1385c9c6</anchor>
      <arglist>(uint64_t intra_process_subscription_id, IntraProcessManagerWeakPtr weak_ipm)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Waitable::SharedPtr</type>
      <name>get_intra_process_waitable</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a1c46f29106172b169769900e1ba63341</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>exchange_in_use_by_wait_set_state</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>afb36042e8da7c498f881b7581feba649</anchor>
      <arglist>(void *pointer_to_subscription_part, bool in_use_state)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>add_event_handler</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a9f57c010a428149762b8165ca5438619</anchor>
      <arglist>(const EventCallbackT &amp;callback, const rcl_subscription_event_type_t event_type)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>default_incompatible_qos_callback</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a537c2376954a0a29aa8a5ef3f1cfe9f7</anchor>
      <arglist>(QOSRequestedIncompatibleQoSInfo &amp;info) const</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>bool</type>
      <name>matches_any_intra_process_publishers</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a99ade691372f06d72bbe36fd133b260b</anchor>
      <arglist>(const rmw_gid_t *sender_gid) const</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::node_interfaces::NodeBaseInterface *const</type>
      <name>node_base_</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a020f78aa2f16e4f01be1a6dc6826d32c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rcl_node_t &gt;</type>
      <name>node_handle_</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a83fe91f64668b491050809168da7b101</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rcl_subscription_t &gt;</type>
      <name>subscription_handle_</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a86dccb346d0f8442dd7add518bcbc648</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rcl_subscription_t &gt;</type>
      <name>intra_process_subscription_handle_</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>ad7897b9def45164032b896f1f685ef24</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::vector&lt; std::shared_ptr&lt; rclcpp::QOSEventHandlerBase &gt; &gt;</type>
      <name>event_handlers_</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a10995cc2fc3565a8416797f0d579e4d6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>bool</type>
      <name>use_intra_process_</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a3e2b849f3a01c83d1ae56d372c87a35d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>IntraProcessManagerWeakPtr</type>
      <name>weak_ipm_</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a9a63c78489d556b95530b722ae80a1d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>uint64_t</type>
      <name>intra_process_subscription_id_</name>
      <anchorfile>classrclcpp_1_1SubscriptionBase.html</anchorfile>
      <anchor>a56e75ff6f8331e308cdb0c96fda52706</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::DynamicStorage::SubscriptionEntry</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1SubscriptionEntry.html</filename>
    <member kind="function">
      <type></type>
      <name>SubscriptionEntry</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1SubscriptionEntry.html</anchorfile>
      <anchor>afc50f1f3bf396bd8c18e16ba62eda005</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; subscription_in=nullptr, const rclcpp::SubscriptionWaitSetMask &amp;mask_in={})</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>reset</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1SubscriptionEntry.html</anchorfile>
      <anchor>a043f4d6898b9fe97c90dbebacfd37957</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; rclcpp::SubscriptionBase &gt;</type>
      <name>subscription</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1SubscriptionEntry.html</anchorfile>
      <anchor>a276ec8daa058e8f905a1d218ba0bd980</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::SubscriptionWaitSetMask</type>
      <name>mask</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1SubscriptionEntry.html</anchorfile>
      <anchor>a47eac1f9bef34c35c887a1baa469e0ad</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::StaticStorage::SubscriptionEntry</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1StaticStorage_1_1SubscriptionEntry.html</filename>
    <member kind="function">
      <type></type>
      <name>SubscriptionEntry</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage_1_1SubscriptionEntry.html</anchorfile>
      <anchor>a4028878c21cdbd2dccecfc6f04263166</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; subscription_in=nullptr, rclcpp::SubscriptionWaitSetMask mask_in={})</arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; rclcpp::SubscriptionBase &gt;</type>
      <name>subscription</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage_1_1SubscriptionEntry.html</anchorfile>
      <anchor>a4501b30d362e7ea2969758c2b5d60b15</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::SubscriptionWaitSetMask</type>
      <name>mask</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1StaticStorage_1_1SubscriptionEntry.html</anchorfile>
      <anchor>a2209b3dd8b2a27e51da0c3afd8575e48</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::SubscriptionEventCallbacks</name>
    <filename>structrclcpp_1_1SubscriptionEventCallbacks.html</filename>
    <member kind="variable">
      <type>QOSDeadlineRequestedCallbackType</type>
      <name>deadline_callback</name>
      <anchorfile>structrclcpp_1_1SubscriptionEventCallbacks.html</anchorfile>
      <anchor>a373f0ff83122110b3c1f8dc153cce5d1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>QOSLivelinessChangedCallbackType</type>
      <name>liveliness_callback</name>
      <anchorfile>structrclcpp_1_1SubscriptionEventCallbacks.html</anchorfile>
      <anchor>a9c2b0a9667f0938124165a3c89d45630</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>QOSRequestedIncompatibleQoSCallbackType</type>
      <name>incompatible_qos_callback</name>
      <anchorfile>structrclcpp_1_1SubscriptionEventCallbacks.html</anchorfile>
      <anchor>a8ce1b0cedb6fd8a48b35a04d71b4c28e</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::SubscriptionFactory</name>
    <filename>structrclcpp_1_1SubscriptionFactory.html</filename>
    <member kind="typedef">
      <type>std::function&lt; rclcpp::SubscriptionBase::SharedPtr(rclcpp::node_interfaces::NodeBaseInterface *node_base, const std::string &amp;topic_name, const rclcpp::QoS &amp;qos)&gt;</type>
      <name>SubscriptionFactoryFunction</name>
      <anchorfile>structrclcpp_1_1SubscriptionFactory.html</anchorfile>
      <anchor>a5c1db345c95efb566678403e953aaa6c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const SubscriptionFactoryFunction</type>
      <name>create_typed_subscription</name>
      <anchorfile>structrclcpp_1_1SubscriptionFactory.html</anchorfile>
      <anchor>ac6d3b8c487b0c5965b95329b5a3e12fa</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::experimental::SubscriptionIntraProcess</name>
    <filename>classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <base>rclcpp::experimental::SubscriptionIntraProcessBase</base>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; MessageT, Alloc &gt;</type>
      <name>MessageAllocTraits</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html</anchorfile>
      <anchor>ab9ade12f7ca40a7351aff1370e26202d</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename MessageAllocTraits::allocator_type</type>
      <name>MessageAlloc</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html</anchorfile>
      <anchor>a69854025f76f0912c8838f4d1270ac4f</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; const MessageT &gt;</type>
      <name>ConstMessageSharedPtr</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html</anchorfile>
      <anchor>ae685883b7ed302498106ca81598ff54e</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::unique_ptr&lt; MessageT, Deleter &gt;</type>
      <name>MessageUniquePtr</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html</anchorfile>
      <anchor>a3f49abbfb0151044c2624e442f2f56c5</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename rclcpp::experimental::buffers::IntraProcessBuffer&lt; MessageT, Alloc, Deleter &gt;::UniquePtr</type>
      <name>BufferUniquePtr</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html</anchorfile>
      <anchor>a2d3522182c12cafc9982473d271a15c7</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SubscriptionIntraProcess</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html</anchorfile>
      <anchor>a8f8cea097df51c8672914ffe416e04e6</anchor>
      <arglist>(AnySubscriptionCallback&lt; CallbackMessageT, Alloc &gt; callback, std::shared_ptr&lt; Alloc &gt; allocator, rclcpp::Context::SharedPtr context, const std::string &amp;topic_name, rmw_qos_profile_t qos_profile, rclcpp::IntraProcessBufferType buffer_type)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_ready</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html</anchorfile>
      <anchor>a2d9ecb0aa914b25012756291ca8b8713</anchor>
      <arglist>(rcl_wait_set_t *wait_set)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>execute</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html</anchorfile>
      <anchor>ad00b43e9084547b575eed688c8a9bda7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>provide_intra_process_message</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html</anchorfile>
      <anchor>a8ebfd7d0da3cbb1b7682bff598f1d782</anchor>
      <arglist>(ConstMessageSharedPtr message)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>provide_intra_process_message</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html</anchorfile>
      <anchor>a11d4eca8c62e25f492afa16364233147</anchor>
      <arglist>(MessageUniquePtr message)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>use_take_shared_method</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html</anchorfile>
      <anchor>a006ad8b8c02fd73ffbb84d186414336c</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::experimental::SubscriptionIntraProcessBase</name>
    <filename>classrclcpp_1_1experimental_1_1SubscriptionIntraProcessBase.html</filename>
    <base>rclcpp::Waitable</base>
    <member kind="function">
      <type></type>
      <name>SubscriptionIntraProcessBase</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcessBase.html</anchorfile>
      <anchor>a9a73d240a3b2604f1df8c29f0d51585c</anchor>
      <arglist>(const std::string &amp;topic_name, rmw_qos_profile_t qos_profile)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~SubscriptionIntraProcessBase</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcessBase.html</anchorfile>
      <anchor>a90c12fbcc8c152544b346121c39e73c0</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function">
      <type>size_t</type>
      <name>get_number_of_ready_guard_conditions</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcessBase.html</anchorfile>
      <anchor>a4450e13e43a3959d1a65db2afd1d804b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>add_to_wait_set</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcessBase.html</anchorfile>
      <anchor>a1c8f01cf56a876c216857f9bfadd0f7f</anchor>
      <arglist>(rcl_wait_set_t *wait_set)</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>is_ready</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcessBase.html</anchorfile>
      <anchor>a6317b609f3ce9631f5750e73eebe3d17</anchor>
      <arglist>(rcl_wait_set_t *wait_set)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>execute</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcessBase.html</anchorfile>
      <anchor>a1f89e0f78c275f43217fe2d02725d48b</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>use_take_shared_method</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcessBase.html</anchorfile>
      <anchor>ab69ddb435daba3fe30797ce971766524</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_topic_name</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcessBase.html</anchorfile>
      <anchor>ae4834d68b40a4d5700b5b8a4f1a5222c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rmw_qos_profile_t</type>
      <name>get_actual_qos</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcessBase.html</anchorfile>
      <anchor>a7edf5f97ca33e790e79e5f19cff8e025</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::recursive_mutex</type>
      <name>reentrant_mutex_</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcessBase.html</anchorfile>
      <anchor>a03e2af429ddc1310b2377091f547e858</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rcl_guard_condition_t</type>
      <name>gc_</name>
      <anchorfile>classrclcpp_1_1experimental_1_1SubscriptionIntraProcessBase.html</anchorfile>
      <anchor>a20b4d76866868aa2c618aef518eee75b</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::SubscriptionOptionsBase</name>
    <filename>structrclcpp_1_1SubscriptionOptionsBase.html</filename>
    <class kind="struct">rclcpp::SubscriptionOptionsBase::TopicStatisticsOptions</class>
    <member kind="variable">
      <type>SubscriptionEventCallbacks</type>
      <name>event_callbacks</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsBase.html</anchorfile>
      <anchor>a614dd91e0339550c3b48fdae7d1482bb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>use_default_callbacks</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsBase.html</anchorfile>
      <anchor>a8dc942e832550210e146109c05460bdc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>ignore_local_publications</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsBase.html</anchorfile>
      <anchor>a96117966ebfb642656ad288858683360</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::CallbackGroup::SharedPtr</type>
      <name>callback_group</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsBase.html</anchorfile>
      <anchor>ab3998ee55c81d8035a0bd4b574200218</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>IntraProcessSetting</type>
      <name>use_intra_process_comm</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsBase.html</anchorfile>
      <anchor>a31c52d6d41971147b08926948f32f31a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>IntraProcessBufferType</type>
      <name>intra_process_buffer_type</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsBase.html</anchorfile>
      <anchor>ac14283c2bfe828918b93270cf9011f2a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; rclcpp::detail::RMWImplementationSpecificSubscriptionPayload &gt;</type>
      <name>rmw_implementation_payload</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsBase.html</anchorfile>
      <anchor>a5a063b9209e95facf81d22e1c639831c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>TopicStatisticsOptions</type>
      <name>topic_stats_options</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsBase.html</anchorfile>
      <anchor>a56e5fef41e5c16fd9e95b7cef9fad5e3</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::SubscriptionOptionsWithAllocator</name>
    <filename>structrclcpp_1_1SubscriptionOptionsWithAllocator.html</filename>
    <templarg></templarg>
    <base>rclcpp::SubscriptionOptionsBase</base>
    <member kind="function">
      <type></type>
      <name>SubscriptionOptionsWithAllocator</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsWithAllocator.html</anchorfile>
      <anchor>a542b539a197df40c71d09c040aeded6a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SubscriptionOptionsWithAllocator</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsWithAllocator.html</anchorfile>
      <anchor>a14fe4e7dafed03e9559c95d562189ee0</anchor>
      <arglist>(const SubscriptionOptionsBase &amp;subscription_options_base)</arglist>
    </member>
    <member kind="function">
      <type>rcl_subscription_options_t</type>
      <name>to_rcl_subscription_options</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsWithAllocator.html</anchorfile>
      <anchor>ab2bc5d54e007194575937548a2b809a4</anchor>
      <arglist>(const rclcpp::QoS &amp;qos) const</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; Allocator &gt;</type>
      <name>get_allocator</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsWithAllocator.html</anchorfile>
      <anchor>a44212259b80c323e0ab2549b4577ca8f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; Allocator &gt;</type>
      <name>allocator</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsWithAllocator.html</anchorfile>
      <anchor>a38dfcf45affafe9fb74d3a782f0aba47</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>SubscriptionOptionsWithAllocator&lt; std::allocator&lt; void &gt; &gt;</name>
    <filename>structrclcpp_1_1SubscriptionOptionsWithAllocator.html</filename>
    <base>rclcpp::SubscriptionOptionsBase</base>
    <member kind="function">
      <type></type>
      <name>SubscriptionOptionsWithAllocator</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsWithAllocator.html</anchorfile>
      <anchor>a542b539a197df40c71d09c040aeded6a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SubscriptionOptionsWithAllocator</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsWithAllocator.html</anchorfile>
      <anchor>a14fe4e7dafed03e9559c95d562189ee0</anchor>
      <arglist>(const SubscriptionOptionsBase &amp;subscription_options_base)</arglist>
    </member>
    <member kind="function">
      <type>rcl_subscription_options_t</type>
      <name>to_rcl_subscription_options</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsWithAllocator.html</anchorfile>
      <anchor>ab2bc5d54e007194575937548a2b809a4</anchor>
      <arglist>(const rclcpp::QoS &amp;qos) const</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; std::allocator&lt; void &gt; &gt;</type>
      <name>get_allocator</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsWithAllocator.html</anchorfile>
      <anchor>a44212259b80c323e0ab2549b4577ca8f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; std::allocator&lt; void &gt; &gt;</type>
      <name>allocator</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsWithAllocator.html</anchorfile>
      <anchor>a38dfcf45affafe9fb74d3a782f0aba47</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::topic_statistics::SubscriptionTopicStatistics</name>
    <filename>classrclcpp_1_1topic__statistics_1_1SubscriptionTopicStatistics.html</filename>
    <templarg></templarg>
    <member kind="function">
      <type></type>
      <name>SubscriptionTopicStatistics</name>
      <anchorfile>classrclcpp_1_1topic__statistics_1_1SubscriptionTopicStatistics.html</anchorfile>
      <anchor>a3e9a4f00d47cf63dc05bc353c6094b12</anchor>
      <arglist>(const std::string &amp;node_name, rclcpp::Publisher&lt; statistics_msgs::msg::MetricsMessage &gt;::SharedPtr publisher)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~SubscriptionTopicStatistics</name>
      <anchorfile>classrclcpp_1_1topic__statistics_1_1SubscriptionTopicStatistics.html</anchorfile>
      <anchor>a2d93709c9e1d17afd5f8bdce5f286038</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>handle_message</name>
      <anchorfile>classrclcpp_1_1topic__statistics_1_1SubscriptionTopicStatistics.html</anchorfile>
      <anchor>a83c4fa59f880289b3d09d326d1e8f5b3</anchor>
      <arglist>(const CallbackMessageT &amp;received_message, const rclcpp::Time now_nanoseconds) const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_publisher_timer</name>
      <anchorfile>classrclcpp_1_1topic__statistics_1_1SubscriptionTopicStatistics.html</anchorfile>
      <anchor>a951ef59b6ba60d9a4a0a2699e75dd062</anchor>
      <arglist>(rclcpp::TimerBase::SharedPtr publisher_timer)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>publish_message</name>
      <anchorfile>classrclcpp_1_1topic__statistics_1_1SubscriptionTopicStatistics.html</anchorfile>
      <anchor>a5228b9eccfe0c3ab235c94a55ffa782a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::vector&lt; StatisticData &gt;</type>
      <name>get_current_collector_data</name>
      <anchorfile>classrclcpp_1_1topic__statistics_1_1SubscriptionTopicStatistics.html</anchorfile>
      <anchor>ac36333fe4dae4c8e6f3a95697a344a61</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::SubscriptionWaitSetMask</name>
    <filename>classrclcpp_1_1SubscriptionWaitSetMask.html</filename>
    <member kind="variable">
      <type>bool</type>
      <name>include_subscription</name>
      <anchorfile>classrclcpp_1_1SubscriptionWaitSetMask.html</anchorfile>
      <anchor>aa5840b846a38eedb8a17274928f49062</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>include_events</name>
      <anchorfile>classrclcpp_1_1SubscriptionWaitSetMask.html</anchorfile>
      <anchor>a86c9c0fd0008b0684d3abfa4508cc6db</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>include_intra_process_waitable</name>
      <anchorfile>classrclcpp_1_1SubscriptionWaitSetMask.html</anchorfile>
      <anchor>ab24d6ec65f38f5c67f30484e2d85fb55</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::detail::SynchronizationPolicyCommon</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1detail_1_1SynchronizationPolicyCommon.html</filename>
    <member kind="function" protection="protected">
      <type></type>
      <name>SynchronizationPolicyCommon</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1SynchronizationPolicyCommon.html</anchorfile>
      <anchor>a4ae4a3e1bac458cb94724f40fdfcf5ba</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>~SynchronizationPolicyCommon</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1SynchronizationPolicyCommon.html</anchorfile>
      <anchor>ad3db9778294edec0f3f81fb30365a085</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::function&lt; bool()&gt;</type>
      <name>create_loop_predicate</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1SynchronizationPolicyCommon.html</anchorfile>
      <anchor>a044f88e4091270fab99a6ebc2409a980</anchor>
      <arglist>(std::chrono::nanoseconds time_to_wait_ns, std::chrono::steady_clock::time_point start)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>std::chrono::nanoseconds</type>
      <name>calculate_time_left_to_wait</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1SynchronizationPolicyCommon.html</anchorfile>
      <anchor>a6fddf182dc8a80190ee128d6297feaa7</anchor>
      <arglist>(std::chrono::nanoseconds original_time_to_wait_ns, std::chrono::steady_clock::time_point start)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::SyncParametersClient</name>
    <filename>classrclcpp_1_1SyncParametersClient.html</filename>
    <member kind="function">
      <type></type>
      <name>SyncParametersClient</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>ad7d9189ce6b12fbe0d23890ed61bb928</anchor>
      <arglist>(rclcpp::Node::SharedPtr node, const std::string &amp;remote_node_name=&quot;&quot;, const rmw_qos_profile_t &amp;qos_profile=rmw_qos_profile_parameters)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SyncParametersClient</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>a586e5d2c9f9da9f0bb6b8968bb81983e</anchor>
      <arglist>(rclcpp::Executor::SharedPtr executor, rclcpp::Node::SharedPtr node, const std::string &amp;remote_node_name=&quot;&quot;, const rmw_qos_profile_t &amp;qos_profile=rmw_qos_profile_parameters)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SyncParametersClient</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>a4cc16ad1b2fac5d9f40fae2bf16d20bf</anchor>
      <arglist>(rclcpp::Node *node, const std::string &amp;remote_node_name=&quot;&quot;, const rmw_qos_profile_t &amp;qos_profile=rmw_qos_profile_parameters)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SyncParametersClient</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>ab4002adc3163ea49dbfd65e610ed090b</anchor>
      <arglist>(rclcpp::Executor::SharedPtr executor, rclcpp::Node *node, const std::string &amp;remote_node_name=&quot;&quot;, const rmw_qos_profile_t &amp;qos_profile=rmw_qos_profile_parameters)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>SyncParametersClient</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>ab6e8bb794d4988377cbb62ccf4d8fd1a</anchor>
      <arglist>(rclcpp::Executor::SharedPtr executor, const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface, const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface, const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface, const std::string &amp;remote_node_name=&quot;&quot;, const rmw_qos_profile_t &amp;qos_profile=rmw_qos_profile_parameters)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rclcpp::Parameter &gt;</type>
      <name>get_parameters</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>a4d95ea47bd9fa8df28a8c9b87810d25c</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;parameter_names)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>has_parameter</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>acc3e72472346d064411a200e8406f6bc</anchor>
      <arglist>(const std::string &amp;parameter_name)</arglist>
    </member>
    <member kind="function">
      <type>T</type>
      <name>get_parameter_impl</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>aba4b159e659a7ee18f720385b7eeb826</anchor>
      <arglist>(const std::string &amp;parameter_name, std::function&lt; T()&gt; parameter_not_found_handler)</arglist>
    </member>
    <member kind="function">
      <type>T</type>
      <name>get_parameter</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>a16c971b86fb286de6c1f0a0b0209621b</anchor>
      <arglist>(const std::string &amp;parameter_name, const T &amp;default_value)</arglist>
    </member>
    <member kind="function">
      <type>T</type>
      <name>get_parameter</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>a794bb58dd7c179320664403c1cd3f250</anchor>
      <arglist>(const std::string &amp;parameter_name)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rclcpp::ParameterType &gt;</type>
      <name>get_parameter_types</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>a0468fe0beb60646cd3cc88a4c8a0e239</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;parameter_names)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; rcl_interfaces::msg::SetParametersResult &gt;</type>
      <name>set_parameters</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>a0df5d7d7e372e8d25c3e973b484addf1</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameters)</arglist>
    </member>
    <member kind="function">
      <type>rcl_interfaces::msg::SetParametersResult</type>
      <name>set_parameters_atomically</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>a267b96b3d93929404ac8ac4e83214831</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameters)</arglist>
    </member>
    <member kind="function">
      <type>rcl_interfaces::msg::ListParametersResult</type>
      <name>list_parameters</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>ac8e38efd59a753b39a066d327b491a71</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;parameter_prefixes, uint64_t depth)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Subscription&lt; rcl_interfaces::msg::ParameterEvent &gt;::SharedPtr</type>
      <name>on_parameter_event</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>ae5d2606ffec3872cf52cc532b34a74bc</anchor>
      <arglist>(CallbackT &amp;&amp;callback)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>service_is_ready</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>a7d424b901082cd9521ca9e6a9066b527</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>wait_for_service</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>ae0c9405bfa1082a4fe21778fb160edbf</anchor>
      <arglist>(std::chrono::duration&lt; RepT, RatioT &gt; timeout=std::chrono::duration&lt; RepT, RatioT &gt;(-1))</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static rclcpp::Subscription&lt; rcl_interfaces::msg::ParameterEvent &gt;::SharedPtr</type>
      <name>on_parameter_event</name>
      <anchorfile>classrclcpp_1_1SyncParametersClient.html</anchorfile>
      <anchor>ae3812a7497851cc808effefaff073d9b</anchor>
      <arglist>(NodeT &amp;&amp;node, CallbackT &amp;&amp;callback)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::SystemDefaultsQoS</name>
    <filename>classrclcpp_1_1SystemDefaultsQoS.html</filename>
    <base>rclcpp::QoS</base>
    <member kind="function">
      <type></type>
      <name>SystemDefaultsQoS</name>
      <anchorfile>classrclcpp_1_1SystemDefaultsQoS.html</anchorfile>
      <anchor>a6513ffbc234675ff4f798b162c983cca</anchor>
      <arglist>(const QoSInitialization &amp;qos_initialization=(QoSInitialization::from_rmw(rmw_qos_profile_system_default)))</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::ThreadSafeSynchronization</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</filename>
    <base>rclcpp::wait_set_policies::detail::SynchronizationPolicyCommon</base>
    <member kind="function" protection="protected">
      <type></type>
      <name>ThreadSafeSynchronization</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a25b1c44bc5dce6bc5c5d526fe9bdbfe6</anchor>
      <arglist>(rclcpp::Context::SharedPtr context)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>~ThreadSafeSynchronization</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a523ece2f5bfb414f06e21d24ad37085e</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>const std::array&lt; std::shared_ptr&lt; rclcpp::GuardCondition &gt;, 1 &gt; &amp;</type>
      <name>get_extra_guard_conditions</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a04e33f5e72e764200d5c3a1440e0f703</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>interrupt_waiting_wait_set</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>af3b42f685a6bccc9851cbdd6db5c25eb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_add_subscription</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>af2c6d973fd042b5f8275fe6b74f5f367</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; &amp;&amp;subscription, const rclcpp::SubscriptionWaitSetMask &amp;mask, std::function&lt; void(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; &amp;&amp;, const rclcpp::SubscriptionWaitSetMask &amp;) &gt; add_subscription_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_remove_subscription</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a5fc741f8ac6bc89a428b955abaee7ebb</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; &amp;&amp;subscription, const rclcpp::SubscriptionWaitSetMask &amp;mask, std::function&lt; void(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; &amp;&amp;, const rclcpp::SubscriptionWaitSetMask &amp;) &gt; remove_subscription_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_add_guard_condition</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>ae4e52f7bb0242ca7c8042dfc69c6463e</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::GuardCondition &gt; &amp;&amp;guard_condition, std::function&lt; void(std::shared_ptr&lt; rclcpp::GuardCondition &gt; &amp;&amp;)&gt; add_guard_condition_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_remove_guard_condition</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a264abc9f24d8119044081294c42bda3c</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::GuardCondition &gt; &amp;&amp;guard_condition, std::function&lt; void(std::shared_ptr&lt; rclcpp::GuardCondition &gt; &amp;&amp;)&gt; remove_guard_condition_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_add_timer</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a5e9b94596a25ad3b42ce31b12bb5d55d</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::TimerBase &gt; &amp;&amp;timer, std::function&lt; void(std::shared_ptr&lt; rclcpp::TimerBase &gt; &amp;&amp;)&gt; add_timer_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_remove_timer</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a4caed452ff0c30dcbc0d665bc2a68f10</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::TimerBase &gt; &amp;&amp;timer, std::function&lt; void(std::shared_ptr&lt; rclcpp::TimerBase &gt; &amp;&amp;)&gt; remove_timer_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_add_client</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>ad9230d1ce4716b4a10126cf96fa14a43</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ClientBase &gt; &amp;&amp;client, std::function&lt; void(std::shared_ptr&lt; rclcpp::ClientBase &gt; &amp;&amp;)&gt; add_client_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_remove_client</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a909f0ce70321db5d28ed395a95145a4e</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ClientBase &gt; &amp;&amp;client, std::function&lt; void(std::shared_ptr&lt; rclcpp::ClientBase &gt; &amp;&amp;)&gt; remove_client_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_add_service</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a09021dae8d26723b873b58f3b732af05</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ServiceBase &gt; &amp;&amp;service, std::function&lt; void(std::shared_ptr&lt; rclcpp::ServiceBase &gt; &amp;&amp;)&gt; add_service_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_remove_service</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a78ed50528a2588f7dc8cb490817c27cb</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ServiceBase &gt; &amp;&amp;service, std::function&lt; void(std::shared_ptr&lt; rclcpp::ServiceBase &gt; &amp;&amp;)&gt; remove_service_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_add_waitable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>abe7ae35be5ec8d7fe9c723ed5250403a</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Waitable &gt; &amp;&amp;waitable, std::shared_ptr&lt; void &gt; &amp;&amp;associated_entity, std::function&lt; void(std::shared_ptr&lt; rclcpp::Waitable &gt; &amp;&amp;, std::shared_ptr&lt; void &gt; &amp;&amp;) &gt; add_waitable_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_remove_waitable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a5a8422aae49fd22e3d355a57c4c025eb</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Waitable &gt; &amp;&amp;waitable, std::function&lt; void(std::shared_ptr&lt; rclcpp::Waitable &gt; &amp;&amp;)&gt; remove_waitable_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_prune_deleted_entities</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>aa6e9bacca0d4d6b401e8f559268358d1</anchor>
      <arglist>(std::function&lt; void()&gt; prune_deleted_entities_function)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>WaitResultT</type>
      <name>sync_wait</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>aea49507d8d656db40f0f8f6fccb35b61</anchor>
      <arglist>(std::chrono::nanoseconds time_to_wait_ns, std::function&lt; void()&gt; rebuild_rcl_wait_set, std::function&lt; rcl_wait_set_t &amp;()&gt; get_rcl_wait_set, std::function&lt; WaitResultT(WaitResultKind wait_result_kind)&gt; create_wait_result)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_wait_result_acquire</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a2703f813939a4f08096c7766b081dcd7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>sync_wait_result_release</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>ab63bf58452dcf7f2bb68e9c5f1fc145d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::array&lt; std::shared_ptr&lt; rclcpp::GuardCondition &gt;, 1 &gt;</type>
      <name>extra_guard_conditions_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a485c09f6a2bf2dc7e67e2c33fa05e204</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock</type>
      <name>wprw_lock_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1ThreadSafeSynchronization.html</anchorfile>
      <anchor>a0eb1155ea9c0c6457efa5ec278fd3d31</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Time</name>
    <filename>classrclcpp_1_1Time.html</filename>
    <member kind="function">
      <type></type>
      <name>Time</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>aff05201cda9688a4d2f9a65a62605c89</anchor>
      <arglist>(int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t clock_type=RCL_SYSTEM_TIME)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Time</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>a683d5111f099d16532f10fcd82e8f5af</anchor>
      <arglist>(int64_t nanoseconds=0, rcl_clock_type_t clock_type=RCL_SYSTEM_TIME)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Time</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>a2eada5cd50809c16012920025f390435</anchor>
      <arglist>(const Time &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Time</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>a13b8e8297e89516806b18c26747804d5</anchor>
      <arglist>(const builtin_interfaces::msg::Time &amp;time_msg, rcl_clock_type_t clock_type=RCL_ROS_TIME)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Time</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>acea08b9c8f66862cda92a0b7a8a6da38</anchor>
      <arglist>(const rcl_time_point_t &amp;time_point)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Time</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>adc504de161e1af2dcba8b7e3d410f655</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator builtin_interfaces::msg::Time</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>afda7e0adc9691fe4dc4a28007e650b07</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Time &amp;</type>
      <name>operator=</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>a5c9f69a1af3f2775891dd8b1b1e86558</anchor>
      <arglist>(const Time &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>Time &amp;</type>
      <name>operator=</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>ae05b1216797d3b804d0019c4aef15e72</anchor>
      <arglist>(const builtin_interfaces::msg::Time &amp;time_msg)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>ad4282a603bdc3f6bc62de5ae126dd469</anchor>
      <arglist>(const rclcpp::Time &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>ae6dd62063affe85828948ae1a0b9da01</anchor>
      <arglist>(const rclcpp::Time &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>a96739c41b57d508815d59304db62f677</anchor>
      <arglist>(const rclcpp::Time &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;=</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>ab5015ce1cfb4e5b36beaf537662342cb</anchor>
      <arglist>(const rclcpp::Time &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;=</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>a56436c041c86ef63c144fae6b0d06a72</anchor>
      <arglist>(const rclcpp::Time &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>af93b6a178cd5c7c8119267a8be30d05b</anchor>
      <arglist>(const rclcpp::Time &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>Time</type>
      <name>operator+</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>ad5f2119018188eb6539c45e35f166b34</anchor>
      <arglist>(const rclcpp::Duration &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>Duration</type>
      <name>operator-</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>a5f28a2a13e6d6ee0541558c99a0fc558</anchor>
      <arglist>(const rclcpp::Time &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>Time</type>
      <name>operator-</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>aca598d89ae39e5276e94775f8ca91c35</anchor>
      <arglist>(const rclcpp::Duration &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>Time &amp;</type>
      <name>operator+=</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>a50f302f3c12997d403a7bbaa725734b8</anchor>
      <arglist>(const rclcpp::Duration &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>Time &amp;</type>
      <name>operator-=</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>aac43181f053856bb6c2eb3f3f98af8f3</anchor>
      <arglist>(const rclcpp::Duration &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>rcl_time_point_value_t</type>
      <name>nanoseconds</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>a0bab304d7b9e92dc568b069684275fe5</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>seconds</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>a586e6c368730e6847519b7cd928359f5</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rcl_clock_type_t</type>
      <name>get_clock_type</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>ab2e1daf9fbab97e710f8b0725625dabd</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static Time</type>
      <name>max</name>
      <anchorfile>classrclcpp_1_1Time.html</anchorfile>
      <anchor>a576a8423504ae9ea5cc3e28d2404e3a0</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::TimerBase</name>
    <filename>classrclcpp_1_1TimerBase.html</filename>
    <member kind="function">
      <type></type>
      <name>TimerBase</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>a1ff9a1ba664fbafba716642a65b8fd40</anchor>
      <arglist>(Clock::SharedPtr clock, std::chrono::nanoseconds period, rclcpp::Context::SharedPtr context)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~TimerBase</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>adeabdbf910b132010906f4be2e505d5d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cancel</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>aa533702829b0d07cd5a2305b941983dd</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_canceled</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>ae9935e2aec2ef96f7aa116e5825dbd30</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>reset</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>ad8529216283f3139c6e94f315c430b25</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>execute_callback</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>a7ff42a9592b71da94b4ff408c8269587</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; const rcl_timer_t &gt;</type>
      <name>get_timer_handle</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>a75e7b4eaa33774694d8c97d7011e75ac</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::chrono::nanoseconds</type>
      <name>time_until_trigger</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>ac25f2af75c8924a38cf289a61a3ae7e6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>is_steady</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>aa368f48e234a1eb2b266b93ec5d6d18e</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_ready</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>a062f0baa72d229f3cac2f02cbbcec38b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>exchange_in_use_by_wait_set_state</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>a17c7185aaae505fc8a9f9f65e4d2f413</anchor>
      <arglist>(bool in_use_state)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>Clock::SharedPtr</type>
      <name>clock_</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>a20495f02e6c6180bf08c3a43fb53353a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rcl_timer_t &gt;</type>
      <name>timer_handle_</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>a51d32ca5e2ab7c44782cb9214668da36</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::atomic&lt; bool &gt;</type>
      <name>in_use_by_wait_set_</name>
      <anchorfile>classrclcpp_1_1TimerBase.html</anchorfile>
      <anchor>abefecba123408dd640f323ace4a75692</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::TimeSource</name>
    <filename>classrclcpp_1_1TimeSource.html</filename>
    <member kind="function">
      <type></type>
      <name>TimeSource</name>
      <anchorfile>classrclcpp_1_1TimeSource.html</anchorfile>
      <anchor>a230d64ce6cb9dfb6dc29c12a1f931c9b</anchor>
      <arglist>(rclcpp::Node::SharedPtr node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>TimeSource</name>
      <anchorfile>classrclcpp_1_1TimeSource.html</anchorfile>
      <anchor>adebfb68a09ee7b2e9ef5713ca8299ee0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>attachNode</name>
      <anchorfile>classrclcpp_1_1TimeSource.html</anchorfile>
      <anchor>aac1790234799fb579ed338e1390228d5</anchor>
      <arglist>(rclcpp::Node::SharedPtr node)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>attachNode</name>
      <anchorfile>classrclcpp_1_1TimeSource.html</anchorfile>
      <anchor>a5ab02e10b0f6814e2b6afb5bed085a4f</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface, rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface, rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface, rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface, rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface, rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface, rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>detachNode</name>
      <anchorfile>classrclcpp_1_1TimeSource.html</anchorfile>
      <anchor>aa8483e071253c835530e3b08f606927c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>attachClock</name>
      <anchorfile>classrclcpp_1_1TimeSource.html</anchorfile>
      <anchor>a369959ae4208a0d3b4227193b09f36db</anchor>
      <arglist>(rclcpp::Clock::SharedPtr clock)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>detachClock</name>
      <anchorfile>classrclcpp_1_1TimeSource.html</anchorfile>
      <anchor>a3946a912cc01f75c2633901337ec3000</anchor>
      <arglist>(rclcpp::Clock::SharedPtr clock)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~TimeSource</name>
      <anchorfile>classrclcpp_1_1TimeSource.html</anchorfile>
      <anchor>a905a8350590dd22bcada4b100dfc0657</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::TopicEndpointInfo</name>
    <filename>classrclcpp_1_1TopicEndpointInfo.html</filename>
    <member kind="function">
      <type></type>
      <name>TopicEndpointInfo</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>ae2878e4cff26ca6cef5373691f551f5b</anchor>
      <arglist>(const rcl_topic_endpoint_info_t &amp;info)</arglist>
    </member>
    <member kind="function">
      <type>std::string &amp;</type>
      <name>node_name</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>a282c90482cfe83387223254fa54064e8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>node_name</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>acc14e48363f25097bc9a6dc76129072c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::string &amp;</type>
      <name>node_namespace</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>a3e443609a9925dc2aefc89995e8a7953</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>node_namespace</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>a26dc6cba8e04f9a08f974377eb205e9e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::string &amp;</type>
      <name>topic_type</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>a1c2a68b301513689db61f3c9a3088b07</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>topic_type</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>a6da9df7e7f06eb681719cd37eb1d7e7b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::EndpointType &amp;</type>
      <name>endpoint_type</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>a0a04ba71c8c5e18334efab30427cf953</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rclcpp::EndpointType &amp;</type>
      <name>endpoint_type</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>acfdf91ff9e4deda38c155007b51e9bf7</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::array&lt; uint8_t, RMW_GID_STORAGE_SIZE &gt; &amp;</type>
      <name>endpoint_gid</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>a7d3af6fd4e45a3135d430ec5a627f123</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const std::array&lt; uint8_t, RMW_GID_STORAGE_SIZE &gt; &amp;</type>
      <name>endpoint_gid</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>a1c68180a17158975db4f33cdea64fef2</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::QoS &amp;</type>
      <name>qos_profile</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>a61f5900cffcd14b6929320be372da098</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rclcpp::QoS &amp;</type>
      <name>qos_profile</name>
      <anchorfile>classrclcpp_1_1TopicEndpointInfo.html</anchorfile>
      <anchor>a40dfbbf4be0dd2a6b34289e79fdaefa2</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::SubscriptionOptionsBase::TopicStatisticsOptions</name>
    <filename>structrclcpp_1_1SubscriptionOptionsBase_1_1TopicStatisticsOptions.html</filename>
    <member kind="variable">
      <type>TopicStatisticsState</type>
      <name>state</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsBase_1_1TopicStatisticsOptions.html</anchorfile>
      <anchor>ae1bff81a5a6c97526b1e8cd5525897f9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::string</type>
      <name>publish_topic</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsBase_1_1TopicStatisticsOptions.html</anchorfile>
      <anchor>a1b7a80f9069a04f9396c113fffcf4413</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::chrono::milliseconds</type>
      <name>publish_period</name>
      <anchorfile>structrclcpp_1_1SubscriptionOptionsBase_1_1TopicStatisticsOptions.html</anchorfile>
      <anchor>ab59ddf68638e2d2a7087db6a28dcd7de</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::function_traits::tuple_tail</name>
    <filename>structrclcpp_1_1function__traits_1_1tuple__tail.html</filename>
    <templarg></templarg>
  </compound>
  <compound kind="struct">
    <name>rclcpp::function_traits::tuple_tail&lt; std::tuple&lt; Head, Tail ... &gt; &gt;</name>
    <filename>structrclcpp_1_1function__traits_1_1tuple__tail_3_01std_1_1tuple_3_01Head_00_01Tail_01_8_8_8_01_4_01_4.html</filename>
    <templarg></templarg>
    <templarg>Tail</templarg>
    <member kind="typedef">
      <type>std::tuple&lt; Tail ... &gt;</type>
      <name>type</name>
      <anchorfile>structrclcpp_1_1function__traits_1_1tuple__tail_3_01std_1_1tuple_3_01Head_00_01Tail_01_8_8_8_01_4_01_4.html</anchorfile>
      <anchor>ad9223442adb7eeb30c6b7b40a91a9258</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::experimental::buffers::TypedIntraProcessBuffer</name>
    <filename>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <base>IntraProcessBuffer&lt; MessageT, std::allocator&lt; void &gt;, std::default_delete&lt; MessageT &gt; &gt;</base>
    <member kind="typedef">
      <type>allocator::AllocRebind&lt; MessageT, Alloc &gt;</type>
      <name>MessageAllocTraits</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>a6b1cb5bbcc226d3b7e7c81ee03840193</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename MessageAllocTraits::allocator_type</type>
      <name>MessageAlloc</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>acc2183d8bcad9ed39612e8a8451eb1bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::unique_ptr&lt; MessageT, MessageDeleter &gt;</type>
      <name>MessageUniquePtr</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>a5f83d830e97ce907568acc29df94f5fd</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; const MessageT &gt;</type>
      <name>MessageSharedPtr</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>ad6392b3b03f68d13babe49a7aa5b34fa</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>TypedIntraProcessBuffer</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>aa494a363fd5b7dc6eac584619870dbb4</anchor>
      <arglist>(std::unique_ptr&lt; BufferImplementationBase&lt; BufferT &gt;&gt; buffer_impl, std::shared_ptr&lt; Alloc &gt; allocator=nullptr)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~TypedIntraProcessBuffer</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>a9945c69119fca1b056666e2c8291fb8a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_shared</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>ad2288e9007ae47fbfe4359d88f00a6fc</anchor>
      <arglist>(MessageSharedPtr msg) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_unique</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>ad7901cd06ef149b7505a34c7cb1a6eb8</anchor>
      <arglist>(MessageUniquePtr msg) override</arglist>
    </member>
    <member kind="function">
      <type>MessageSharedPtr</type>
      <name>consume_shared</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>a8c751dca12591d59ccd86c4d52c4ef01</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>MessageUniquePtr</type>
      <name>consume_unique</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>ad862fbb5056ee8ea7edaba5d104cb847</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>has_data</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>afd540eb580d8fa97c0eb9864152dd845</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>clear</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>ace8b27c90b526d3d233ec1379e9ca2a5</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>use_take_shared_method</name>
      <anchorfile>classrclcpp_1_1experimental_1_1buffers_1_1TypedIntraProcessBuffer.html</anchorfile>
      <anchor>affd8039a190e274abd92d0df782f062f</anchor>
      <arglist>() const override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::UnimplementedError</name>
    <filename>classrclcpp_1_1exceptions_1_1UnimplementedError.html</filename>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>UnimplementedError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1UnimplementedError.html</anchorfile>
      <anchor>a2b9c792fc83898584dcd2d9b37b177c0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>UnimplementedError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1UnimplementedError.html</anchorfile>
      <anchor>a60729d25675b6788aead22bcb80d33b5</anchor>
      <arglist>(const std::string &amp;msg)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::exceptions::UnknownROSArgsError</name>
    <filename>classrclcpp_1_1exceptions_1_1UnknownROSArgsError.html</filename>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>UnknownROSArgsError</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1UnknownROSArgsError.html</anchorfile>
      <anchor>ad02ad3b1a9416dca98060286e78f9ca9</anchor>
      <arglist>(std::vector&lt; std::string &gt; &amp;&amp;unknown_ros_args_in)</arglist>
    </member>
    <member kind="variable">
      <type>const std::vector&lt; std::string &gt;</type>
      <name>unknown_ros_args</name>
      <anchorfile>classrclcpp_1_1exceptions_1_1UnknownROSArgsError.html</anchorfile>
      <anchor>a2477edde1e2fc54ff70c398cba669d1d</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::UnsupportedEventTypeException</name>
    <filename>classrclcpp_1_1UnsupportedEventTypeException.html</filename>
    <base>rclcpp::exceptions::RCLErrorBase</base>
    <base>std::runtime_error</base>
    <member kind="function">
      <type></type>
      <name>UnsupportedEventTypeException</name>
      <anchorfile>classrclcpp_1_1UnsupportedEventTypeException.html</anchorfile>
      <anchor>adf2e15be4c5c129d552076255c3ceea8</anchor>
      <arglist>(rcl_ret_t ret, const rcl_error_state_t *error_state, const std::string &amp;prefix)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>UnsupportedEventTypeException</name>
      <anchorfile>classrclcpp_1_1UnsupportedEventTypeException.html</anchorfile>
      <anchor>af7c07215964cb2f1a6a5e0334fe8f122</anchor>
      <arglist>(const exceptions::RCLErrorBase &amp;base_exc, const std::string &amp;prefix)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::Waitable</name>
    <filename>classrclcpp_1_1Waitable.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Waitable</name>
      <anchorfile>classrclcpp_1_1Waitable.html</anchorfile>
      <anchor>a775475ffdec36b88279a0d671739cce2</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual size_t</type>
      <name>get_number_of_ready_subscriptions</name>
      <anchorfile>classrclcpp_1_1Waitable.html</anchorfile>
      <anchor>a4f428f90de03558d8e5090b45fa96ac8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual size_t</type>
      <name>get_number_of_ready_timers</name>
      <anchorfile>classrclcpp_1_1Waitable.html</anchorfile>
      <anchor>af46ecd60ac2ddee1a76f280a05155334</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual size_t</type>
      <name>get_number_of_ready_clients</name>
      <anchorfile>classrclcpp_1_1Waitable.html</anchorfile>
      <anchor>aeb7d6a8213736f5a60d9c8f9290866a9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual size_t</type>
      <name>get_number_of_ready_events</name>
      <anchorfile>classrclcpp_1_1Waitable.html</anchorfile>
      <anchor>a441c473cb21623371b8ae0bc4d4ff7ce</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual size_t</type>
      <name>get_number_of_ready_services</name>
      <anchorfile>classrclcpp_1_1Waitable.html</anchorfile>
      <anchor>a49309417506cfee91923ce88ce95b63f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual size_t</type>
      <name>get_number_of_ready_guard_conditions</name>
      <anchorfile>classrclcpp_1_1Waitable.html</anchorfile>
      <anchor>a154b4cee45c24584485586cb4de10ee8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>add_to_wait_set</name>
      <anchorfile>classrclcpp_1_1Waitable.html</anchorfile>
      <anchor>a5f5d16fa6c3e9badafb35fdde5cf81c6</anchor>
      <arglist>(rcl_wait_set_t *wait_set)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>is_ready</name>
      <anchorfile>classrclcpp_1_1Waitable.html</anchorfile>
      <anchor>a7a71a73b57677c3097012a0bb23b83d4</anchor>
      <arglist>(rcl_wait_set_t *wait_set)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>execute</name>
      <anchorfile>classrclcpp_1_1Waitable.html</anchorfile>
      <anchor>ad9dd88c08883a5de0aa7931fbf57c240</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>exchange_in_use_by_wait_set_state</name>
      <anchorfile>classrclcpp_1_1Waitable.html</anchorfile>
      <anchor>aa73eeb074bfbbd858d5c559d6b185ff8</anchor>
      <arglist>(bool in_use_state)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>rclcpp::wait_set_policies::StaticStorage::WaitableEntry</name>
    <filename>structrclcpp_1_1wait__set__policies_1_1StaticStorage_1_1WaitableEntry.html</filename>
    <member kind="function">
      <type></type>
      <name>WaitableEntry</name>
      <anchorfile>structrclcpp_1_1wait__set__policies_1_1StaticStorage_1_1WaitableEntry.html</anchorfile>
      <anchor>afd1177017d09198b040756331b092e22</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Waitable &gt; waitable_in=nullptr, std::shared_ptr&lt; void &gt; associated_entity_in=nullptr) noexcept</arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; rclcpp::Waitable &gt;</type>
      <name>waitable</name>
      <anchorfile>structrclcpp_1_1wait__set__policies_1_1StaticStorage_1_1WaitableEntry.html</anchorfile>
      <anchor>ad962f8559a177e5ae6d984ca054fcc85</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>associated_entity</name>
      <anchorfile>structrclcpp_1_1wait__set__policies_1_1StaticStorage_1_1WaitableEntry.html</anchorfile>
      <anchor>acfee300008107f524b225066e0ab3603</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::DynamicStorage::WaitableEntry</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WaitableEntry.html</filename>
    <member kind="function">
      <type></type>
      <name>WaitableEntry</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WaitableEntry.html</anchorfile>
      <anchor>a534cb0a926943cec1c0356d4bf0fd8d2</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Waitable &gt; waitable_in=nullptr, std::shared_ptr&lt; void &gt; associated_entity_in=nullptr) noexcept</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>reset</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WaitableEntry.html</anchorfile>
      <anchor>af7c77576bb01f9f9e2d2f78cf42d2522</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; rclcpp::Waitable &gt;</type>
      <name>waitable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WaitableEntry.html</anchorfile>
      <anchor>a778ce9560df4d37d5a00af9648073d10</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::shared_ptr&lt; void &gt;</type>
      <name>associated_entity</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WaitableEntry.html</anchorfile>
      <anchor>a5d32cb3266427f16d73854d449cb110c</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::WaitResult</name>
    <filename>classrclcpp_1_1WaitResult.html</filename>
    <templarg></templarg>
    <member kind="function">
      <type>WaitResultKind</type>
      <name>kind</name>
      <anchorfile>classrclcpp_1_1WaitResult.html</anchorfile>
      <anchor>a8eb88c3215500c4e6a16f286b48ed847</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const WaitSetT &amp;</type>
      <name>get_wait_set</name>
      <anchorfile>classrclcpp_1_1WaitResult.html</anchorfile>
      <anchor>a7e1d835874f95fbf115fbf7a70dd02f3</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>WaitSetT &amp;</type>
      <name>get_wait_set</name>
      <anchorfile>classrclcpp_1_1WaitResult.html</anchorfile>
      <anchor>ab502aff2ba917d2c94c65f6551fa66e7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>WaitResult</name>
      <anchorfile>classrclcpp_1_1WaitResult.html</anchorfile>
      <anchor>a1050f0650a090298ebfe6d14e1f68a74</anchor>
      <arglist>(WaitResult &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~WaitResult</name>
      <anchorfile>classrclcpp_1_1WaitResult.html</anchorfile>
      <anchor>a1558f4e2ced60f85d2103d633e029bce</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static WaitResult</type>
      <name>from_ready_wait_result_kind</name>
      <anchorfile>classrclcpp_1_1WaitResult.html</anchorfile>
      <anchor>a4abbbbfd4ef9620d7323162c48491314</anchor>
      <arglist>(WaitSetT &amp;wait_set)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static WaitResult</type>
      <name>from_timeout_wait_result_kind</name>
      <anchorfile>classrclcpp_1_1WaitResult.html</anchorfile>
      <anchor>a682563cd676888d36624a70ef6af7495</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static WaitResult</type>
      <name>from_empty_wait_result_kind</name>
      <anchorfile>classrclcpp_1_1WaitResult.html</anchorfile>
      <anchor>a5c0eda2992415e7855215c3196458798</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::WaitSetTemplate</name>
    <filename>classrclcpp_1_1WaitSetTemplate.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <member kind="function">
      <type></type>
      <name>WaitSetTemplate</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>a53bf27a0dd8ec262e2ac2f36fa0c2b96</anchor>
      <arglist>(const typename StoragePolicy::SubscriptionsIterable &amp;subscriptions={}, const typename StoragePolicy::GuardConditionsIterable &amp;guard_conditions={}, const typename StoragePolicy::TimersIterable &amp;timers={}, const typename StoragePolicy::ClientsIterable &amp;clients={}, const typename StoragePolicy::ServicesIterable &amp;services={}, const typename StoragePolicy::WaitablesIterable &amp;waitables={}, rclcpp::Context::SharedPtr context=rclcpp::contexts::get_global_default_context())</arglist>
    </member>
    <member kind="function">
      <type>const rcl_wait_set_t &amp;</type>
      <name>get_rcl_wait_set</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>af3d1e940574b075583cad733b65e9733</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_subscription</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>a503c56f84276028350dd3d2287418727</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; subscription, rclcpp::SubscriptionWaitSetMask mask={})</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_subscription</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>a87137037fcf21a6a1bcdf0cd3247c843</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; subscription, rclcpp::SubscriptionWaitSetMask mask={})</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_guard_condition</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>a1773c9f262142c76657afbd290888e06</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::GuardCondition &gt; guard_condition)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_guard_condition</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>aa1021de92357096cb92ffbb1e79996a3</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::GuardCondition &gt; guard_condition)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_timer</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>acc813b4cb6f18dd707269097acec26e7</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::TimerBase &gt; timer)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_timer</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>aa3cef3f62dc2c7412c9e530ee48f8376</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::TimerBase &gt; timer)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_client</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>a9669ab3069967f0972d48d2fefee8bcb</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ClientBase &gt; client)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_client</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>a50bb31d6746db9fcc530d26288b59366</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ClientBase &gt; client)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_service</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>a36acee7f542fc5c58fa7064ab4fd55b0</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ServiceBase &gt; service)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_service</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>ada4d08ac5c17cebf9b9fb8d4a7e92bcd</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::ServiceBase &gt; service)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>add_waitable</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>a636ab1de60af6d9659ffa60fa4a700fb</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Waitable &gt; waitable, std::shared_ptr&lt; void &gt; associated_entity=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>remove_waitable</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>a22120ae19bd980fc1029b233be72f2ff</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Waitable &gt; waitable)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>prune_deleted_entities</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>a090483821f8ebb4f45f3d20ee41fe182</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>RCUTILS_WARN_UNUSED WaitResult&lt; WaitSetTemplate &gt;</type>
      <name>wait</name>
      <anchorfile>classrclcpp_1_1WaitSetTemplate.html</anchorfile>
      <anchor>ac040cac545d4c81095352d0a77eb3290</anchor>
      <arglist>(std::chrono::duration&lt; Rep, Period &gt; time_to_wait=std::chrono::duration&lt; Rep, Period &gt;(-1))</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::WallTimer</name>
    <filename>classrclcpp_1_1WallTimer.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <base>GenericTimer&lt; FunctorT &gt;</base>
    <member kind="function">
      <type></type>
      <name>WallTimer</name>
      <anchorfile>classrclcpp_1_1WallTimer.html</anchorfile>
      <anchor>a34cde63851992027d52b9d93d4552ca7</anchor>
      <arglist>(std::chrono::nanoseconds period, FunctorT &amp;&amp;callback, rclcpp::Context::SharedPtr context)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::DynamicStorage::WeakSubscriptionEntry</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakSubscriptionEntry.html</filename>
    <member kind="function">
      <type></type>
      <name>WeakSubscriptionEntry</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakSubscriptionEntry.html</anchorfile>
      <anchor>ace5063f2464471f87e8450f9af6bbda9</anchor>
      <arglist>(const std::shared_ptr&lt; rclcpp::SubscriptionBase &gt; &amp;subscription_in, const rclcpp::SubscriptionWaitSetMask &amp;mask_in) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>WeakSubscriptionEntry</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakSubscriptionEntry.html</anchorfile>
      <anchor>a73ed696f5bc4960db1d5bc770cb18989</anchor>
      <arglist>(const SubscriptionEntry &amp;other)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rclcpp::SubscriptionBase &gt;</type>
      <name>lock</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakSubscriptionEntry.html</anchorfile>
      <anchor>abbb09ec9664dde91c5a912ab9289f128</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>expired</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakSubscriptionEntry.html</anchorfile>
      <anchor>a5c119b2647865415c18f47ac3f3ed654</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="variable">
      <type>std::weak_ptr&lt; rclcpp::SubscriptionBase &gt;</type>
      <name>subscription</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakSubscriptionEntry.html</anchorfile>
      <anchor>a3f040dd3d1608755a5203b683d55a86d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>rclcpp::SubscriptionWaitSetMask</type>
      <name>mask</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakSubscriptionEntry.html</anchorfile>
      <anchor>aa93d9b805cf464990b2d455e180edc96</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::DynamicStorage::WeakWaitableEntry</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakWaitableEntry.html</filename>
    <member kind="function">
      <type></type>
      <name>WeakWaitableEntry</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakWaitableEntry.html</anchorfile>
      <anchor>ae8f4a38a8ad567d338284105869cdc41</anchor>
      <arglist>(const std::shared_ptr&lt; rclcpp::Waitable &gt; &amp;waitable_in, const std::shared_ptr&lt; void &gt; &amp;associated_entity_in) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>WeakWaitableEntry</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakWaitableEntry.html</anchorfile>
      <anchor>a56521cbcbe31e7390c0cfa26f6571225</anchor>
      <arglist>(const WaitableEntry &amp;other)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; rclcpp::Waitable &gt;</type>
      <name>lock</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakWaitableEntry.html</anchorfile>
      <anchor>a9d3ec8d2ab21395fe8fe48762cd09667</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>expired</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakWaitableEntry.html</anchorfile>
      <anchor>a5e8281e319d5570e895c018b5cafb2c0</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="variable">
      <type>std::weak_ptr&lt; rclcpp::Waitable &gt;</type>
      <name>waitable</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakWaitableEntry.html</anchorfile>
      <anchor>ab13678bc23997dac9690c338625304e1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::weak_ptr&lt; void &gt;</type>
      <name>associated_entity</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1DynamicStorage_1_1WeakWaitableEntry.html</anchorfile>
      <anchor>a98c3606e864b0bcb3c5232fa3dbb679e</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock::WriteMutex</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock_1_1WriteMutex.html</filename>
    <member kind="function">
      <type>void</type>
      <name>lock</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock_1_1WriteMutex.html</anchorfile>
      <anchor>a049886510a7ff00c521deb68da6d9748</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>unlock</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock_1_1WriteMutex.html</anchorfile>
      <anchor>aa8cf94506d149d70ba620e6e21c481ee</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>WriteMutex</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock_1_1WriteMutex.html</anchorfile>
      <anchor>adf6db29e5fa9795415115fa68096c1e5</anchor>
      <arglist>(WritePreferringReadWriteLock &amp;parent_lock)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>WritePreferringReadWriteLock &amp;</type>
      <name>parent_lock_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock_1_1WriteMutex.html</anchorfile>
      <anchor>aeb1d2aa924da1d587e7eca41a8d9025e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>friend</type>
      <name>WritePreferringReadWriteLock</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock_1_1WriteMutex.html</anchorfile>
      <anchor>ad36413626f518e6d92242836c57feb46</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock</name>
    <filename>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock.html</filename>
    <class kind="class">rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock::ReadMutex</class>
    <class kind="class">rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock::WriteMutex</class>
    <member kind="function">
      <type></type>
      <name>WritePreferringReadWriteLock</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock.html</anchorfile>
      <anchor>abc763676b1ef57111ab7ba02dff17952</anchor>
      <arglist>(std::function&lt; void()&gt; enter_waiting_function=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>ReadMutex &amp;</type>
      <name>get_read_mutex</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock.html</anchorfile>
      <anchor>a663bb0664988b9c6754b1327078f6704</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>WriteMutex &amp;</type>
      <name>get_write_mutex</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock.html</anchorfile>
      <anchor>ad72f99da9fe1ce7ccaae58b737e653be</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>bool</type>
      <name>reader_active_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock.html</anchorfile>
      <anchor>a94054abda39ee4cf1dad5e682f7324ad</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::size_t</type>
      <name>number_of_writers_waiting_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock.html</anchorfile>
      <anchor>a40eb9d8e2b4dc67598494357268e9186</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>bool</type>
      <name>writer_active_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock.html</anchorfile>
      <anchor>a78e320bd65cbe5ebe9b504449b22e499</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::mutex</type>
      <name>mutex_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock.html</anchorfile>
      <anchor>a4b7e2dc6110d908f602c44f999f14302</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::condition_variable</type>
      <name>condition_variable_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock.html</anchorfile>
      <anchor>a58524cbeaa93356b594b45f0062c8363</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>ReadMutex</type>
      <name>read_mutex_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock.html</anchorfile>
      <anchor>ac790a9680f11d081af241a4336faea79</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>WriteMutex</type>
      <name>write_mutex_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock.html</anchorfile>
      <anchor>a7bb73ca818495131f81564de9c5a3667</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::function&lt; void()&gt;</type>
      <name>enter_waiting_function_</name>
      <anchorfile>classrclcpp_1_1wait__set__policies_1_1detail_1_1WritePreferringReadWriteLock.html</anchorfile>
      <anchor>a8396cf4e28ac6131d9cc2c31392a5bbc</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp</name>
    <filename>namespacerclcpp.html</filename>
    <namespace>rclcpp::allocator</namespace>
    <namespace>rclcpp::callback_group</namespace>
    <namespace>rclcpp::contexts</namespace>
    <namespace>rclcpp::detail</namespace>
    <namespace>rclcpp::exceptions</namespace>
    <namespace>rclcpp::executor</namespace>
    <namespace>rclcpp::executors</namespace>
    <namespace>rclcpp::experimental</namespace>
    <namespace>rclcpp::function_traits</namespace>
    <namespace>rclcpp::graph_listener</namespace>
    <namespace>rclcpp::memory_strategies</namespace>
    <namespace>rclcpp::memory_strategy</namespace>
    <namespace>rclcpp::message_memory_strategy</namespace>
    <namespace>rclcpp::node_interfaces</namespace>
    <namespace>rclcpp::serialization_traits</namespace>
    <namespace>rclcpp::strategies</namespace>
    <namespace>rclcpp::subscription_traits</namespace>
    <namespace>rclcpp::topic_statistics</namespace>
    <namespace>rclcpp::type_support</namespace>
    <namespace>rclcpp::wait_set_policies</namespace>
    <class kind="struct">rclcpp::AnyExecutable</class>
    <class kind="class">rclcpp::AnyServiceCallback</class>
    <class kind="class">rclcpp::AnySubscriptionCallback</class>
    <class kind="class">rclcpp::AsyncParametersClient</class>
    <class kind="class">rclcpp::CallbackGroup</class>
    <class kind="class">rclcpp::Client</class>
    <class kind="class">rclcpp::ClientBase</class>
    <class kind="class">rclcpp::Clock</class>
    <class kind="class">rclcpp::Context</class>
    <class kind="class">rclcpp::ContextAlreadyInitialized</class>
    <class kind="class">rclcpp::Duration</class>
    <class kind="class">rclcpp::Event</class>
    <class kind="class">rclcpp::Executor</class>
    <class kind="struct">rclcpp::ExecutorOptions</class>
    <class kind="class">rclcpp::GenericRate</class>
    <class kind="class">rclcpp::GenericTimer</class>
    <class kind="class">rclcpp::GuardCondition</class>
    <class kind="class">rclcpp::InitOptions</class>
    <class kind="class">rclcpp::JumpHandler</class>
    <class kind="struct">rclcpp::KeepAll</class>
    <class kind="struct">rclcpp::KeepLast</class>
    <class kind="class">rclcpp::LoanedMessage</class>
    <class kind="class">rclcpp::Logger</class>
    <class kind="class">rclcpp::MessageInfo</class>
    <class kind="class">rclcpp::Node</class>
    <class kind="class">rclcpp::NodeOptions</class>
    <class kind="class">rclcpp::Parameter</class>
    <class kind="class">rclcpp::ParameterEventsFilter</class>
    <class kind="class">rclcpp::ParameterEventsQoS</class>
    <class kind="class">rclcpp::ParameterService</class>
    <class kind="class">rclcpp::ParametersQoS</class>
    <class kind="class">rclcpp::ParameterTypeException</class>
    <class kind="class">rclcpp::ParameterValue</class>
    <class kind="class">rclcpp::Publisher</class>
    <class kind="class">rclcpp::PublisherBase</class>
    <class kind="struct">rclcpp::PublisherEventCallbacks</class>
    <class kind="struct">rclcpp::PublisherFactory</class>
    <class kind="struct">rclcpp::PublisherOptionsBase</class>
    <class kind="struct">rclcpp::PublisherOptionsWithAllocator</class>
    <class kind="class">rclcpp::QoS</class>
    <class kind="class">rclcpp::QOSEventHandler</class>
    <class kind="class">rclcpp::QOSEventHandlerBase</class>
    <class kind="struct">rclcpp::QoSInitialization</class>
    <class kind="class">rclcpp::RateBase</class>
    <class kind="struct">rclcpp::ScopeExit</class>
    <class kind="class">rclcpp::SensorDataQoS</class>
    <class kind="class">rclcpp::Serialization</class>
    <class kind="class">rclcpp::SerializationBase</class>
    <class kind="class">rclcpp::SerializedMessage</class>
    <class kind="class">rclcpp::Service</class>
    <class kind="class">rclcpp::ServiceBase</class>
    <class kind="class">rclcpp::ServicesQoS</class>
    <class kind="class">rclcpp::Subscription</class>
    <class kind="class">rclcpp::SubscriptionBase</class>
    <class kind="struct">rclcpp::SubscriptionEventCallbacks</class>
    <class kind="struct">rclcpp::SubscriptionFactory</class>
    <class kind="struct">rclcpp::SubscriptionOptionsBase</class>
    <class kind="struct">rclcpp::SubscriptionOptionsWithAllocator</class>
    <class kind="class">rclcpp::SubscriptionWaitSetMask</class>
    <class kind="class">rclcpp::SyncParametersClient</class>
    <class kind="class">rclcpp::SystemDefaultsQoS</class>
    <class kind="class">rclcpp::Time</class>
    <class kind="class">rclcpp::TimerBase</class>
    <class kind="class">rclcpp::TimeSource</class>
    <class kind="class">rclcpp::TopicEndpointInfo</class>
    <class kind="class">rclcpp::UnsupportedEventTypeException</class>
    <class kind="class">rclcpp::Waitable</class>
    <class kind="class">rclcpp::WaitResult</class>
    <class kind="class">rclcpp::WaitSetTemplate</class>
    <class kind="class">rclcpp::WallTimer</class>
    <member kind="typedef">
      <type>std::unordered_map&lt; std::string, std::vector&lt; Parameter &gt; &gt;</type>
      <name>ParameterMap</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa587f11d0e53c713ccc0addf5132d46a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>PublisherOptionsWithAllocator&lt; std::allocator&lt; void &gt; &gt;</type>
      <name>PublisherOptions</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a9f38e986e5843aa39fc2b9fff9cf927f</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rmw_requested_deadline_missed_status_t</type>
      <name>QOSDeadlineRequestedInfo</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a51f406f0e8e4928bd5490d5b170c2c3b</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rmw_offered_deadline_missed_status_t</type>
      <name>QOSDeadlineOfferedInfo</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aedf85a88396716dfeff77447176ed05c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rmw_liveliness_changed_status_t</type>
      <name>QOSLivelinessChangedInfo</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a144fad513d523f56d77a071740da8d11</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rmw_liveliness_lost_status_t</type>
      <name>QOSLivelinessLostInfo</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a0a89114d63f459ecc69241fa4f0dfc1f</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rmw_offered_qos_incompatible_event_status_t</type>
      <name>QOSOfferedIncompatibleQoSInfo</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>af6174538b0f569e38ba3be09812ad54c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rmw_requested_qos_incompatible_event_status_t</type>
      <name>QOSRequestedIncompatibleQoSInfo</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a26c69f1ce0937b36bc3915e989fb9faf</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(QOSDeadlineRequestedInfo &amp;)&gt;</type>
      <name>QOSDeadlineRequestedCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a742c7a1550bb04d91fc14f92adffd218</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(QOSDeadlineOfferedInfo &amp;)&gt;</type>
      <name>QOSDeadlineOfferedCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>abb6987cb949f1c2992beb9887311a5b6</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(QOSLivelinessChangedInfo &amp;)&gt;</type>
      <name>QOSLivelinessChangedCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a51148cfeddce96db7ad911fa893a4d82</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(QOSLivelinessLostInfo &amp;)&gt;</type>
      <name>QOSLivelinessLostCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a33673fe9ebe74818bc56d517eae84a03</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(QOSOfferedIncompatibleQoSInfo &amp;)&gt;</type>
      <name>QOSOfferedIncompatibleQoSCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a859176cdce7e529ada6dfa9b0c41be24</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(QOSRequestedIncompatibleQoSInfo &amp;)&gt;</type>
      <name>QOSRequestedIncompatibleQoSCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ab7ae5f255085a1ee084a83b7b8ec4ddb</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>GenericRate&lt; std::chrono::system_clock &gt;</type>
      <name>Rate</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>abe6d4a275a0f75ba817c8486f641f4ea</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>GenericRate&lt; std::chrono::steady_clock &gt;</type>
      <name>WallRate</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a9c57364eb16ca720485d170ee5b99cf7</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>SubscriptionOptionsWithAllocator&lt; std::allocator&lt; void &gt; &gt;</type>
      <name>SubscriptionOptions</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa0ce420d67bb40b61156347455d3ad23</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void()&gt;</type>
      <name>VoidCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aba01c682b9c93b3ff844cc36ea80fda1</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::function&lt; void(TimerBase &amp;)&gt;</type>
      <name>TimerCallbackType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa6bb5a91fd8ef745ce261338f803fc9e</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rclcpp::WaitSetTemplate&lt; rclcpp::wait_set_policies::SequentialSynchronization, rclcpp::wait_set_policies::DynamicStorage &gt;</type>
      <name>WaitSet</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ad6fb19c154de27e92430309d2da25ac3</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rclcpp::WaitSetTemplate&lt; rclcpp::wait_set_policies::SequentialSynchronization, rclcpp::wait_set_policies::StaticStorage&lt; NumberOfSubscriptions, NumberOfGuardCondtions, NumberOfTimers, NumberOfClients, NumberOfServices, NumberOfWaitables &gt; &gt;</type>
      <name>StaticWaitSet</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>adb06acf4a5723b1445fa6ed4e8f73374</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rclcpp::WaitSetTemplate&lt; rclcpp::wait_set_policies::ThreadSafeSynchronization, rclcpp::wait_set_policies::DynamicStorage &gt;</type>
      <name>ThreadSafeWaitSet</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>acaec573e71549fd3078644e18e7f7127</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>CallbackGroupType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a0027d5804ef28f0b6fea8eea4195c44a</anchor>
      <arglist></arglist>
      <enumvalue file="namespacerclcpp.html" anchor="a0027d5804ef28f0b6fea8eea4195c44aa0a658d9024420a1c2f737e8881406f7d">MutuallyExclusive</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a0027d5804ef28f0b6fea8eea4195c44aa49816c033d9f3ad7e81fdb953fe3251f">Reentrant</enumvalue>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>FutureReturnCode</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a7b4ff5f1e516740d7e11ea97fe6f5532</anchor>
      <arglist></arglist>
      <enumvalue file="namespacerclcpp.html" anchor="a7b4ff5f1e516740d7e11ea97fe6f5532ad0749aaba8b833466dfcbb0428e4f89c">SUCCESS</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a7b4ff5f1e516740d7e11ea97fe6f5532a658f2cadfdf09b6046246e9314f7cd43">INTERRUPTED</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a7b4ff5f1e516740d7e11ea97fe6f5532a070a0fb40f6c308ab544b227660aadff">TIMEOUT</enumvalue>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>IntraProcessBufferType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a53c5da68d5964c6bd7894afe4a76a92b</anchor>
      <arglist></arglist>
      <enumvalue file="namespacerclcpp.html" anchor="a53c5da68d5964c6bd7894afe4a76a92ba309589a0fa2f1ec5e6b286ac5e8b6ac8">SharedPtr</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a53c5da68d5964c6bd7894afe4a76a92ba8c4322b401772928915c5a3ada1304d5">UniquePtr</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a53c5da68d5964c6bd7894afe4a76a92ba50c8a640832cc7a38533a3d5d3da60df">CallbackDefault</enumvalue>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>IntraProcessSetting</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a2ebc5fdfcb7e3f025f8ebc2fe1d10fe5</anchor>
      <arglist></arglist>
      <enumvalue file="namespacerclcpp.html" anchor="a2ebc5fdfcb7e3f025f8ebc2fe1d10fe5a2faec1f9f8cc7f8f40d521c4dd574f49">Enable</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a2ebc5fdfcb7e3f025f8ebc2fe1d10fe5abcfaccebf745acfd5e75351095a5394a">Disable</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a2ebc5fdfcb7e3f025f8ebc2fe1d10fe5a5e7bd84eadd196f52e8320680fa1c7cf">NodeDefault</enumvalue>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>EndpointType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ad47a956a91d1787241c564827be18aa5</anchor>
      <arglist></arglist>
      <enumvalue file="namespacerclcpp.html" anchor="ad47a956a91d1787241c564827be18aa5a4bbb8f967da6d1a610596d7257179c2b">Invalid</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="ad47a956a91d1787241c564827be18aa5a32c73be0cb175da278c8e2af0811b0d1">Publisher</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="ad47a956a91d1787241c564827be18aa5a787ad0b7a17de4ad6b1711bbf8d79fcb">Subscription</enumvalue>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>ParameterType</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_NOT_SET</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784adefddb470d1054265245a0c5dbeccf4e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_BOOL</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784a525d4dd60c307e7fb76d57cdeafce8b1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_INTEGER</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784a3a70ea9e9528656834106cf8a647e878</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_DOUBLE</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784a21dd7d8db14cc732692bea98eb073c20</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_STRING</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784a1ed2b44e2043e3cf925a1891e19fd73a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_BYTE_ARRAY</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784aac6f099e5c9b63fd51a75ae587a98271</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_BOOL_ARRAY</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784ab14cef41f8d8d14f9beef95eeb6dd91d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_INTEGER_ARRAY</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784a5b5c50ef3c04a90a59a31b3ad20a4495</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_DOUBLE_ARRAY</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784af685d20a1a39f3531732919ee99f44ab</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PARAMETER_STRING_ARRAY</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1b307a4b7368b68f325812cd2aeb8784ab19e036eb99725754e11c35c6f8d9eb1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>TopicStatisticsState</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1faa0b05040d3cf0f74c231b6800bffc</anchor>
      <arglist></arglist>
      <enumvalue file="namespacerclcpp.html" anchor="a1faa0b05040d3cf0f74c231b6800bffca2faec1f9f8cc7f8f40d521c4dd574f49">Enable</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a1faa0b05040d3cf0f74c231b6800bffcabcfaccebf745acfd5e75351095a5394a">Disable</enumvalue>
      <enumvalue file="namespacerclcpp.html" anchor="a1faa0b05040d3cf0f74c231b6800bffca5e7bd84eadd196f52e8320680fa1c7cf">NodeDefault</enumvalue>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>WaitResultKind</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aac834535a6eab608968bbb44d9773bb1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Ready</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aac834535a6eab608968bbb44d9773bb1ad46050bbb7cc1d8c1836bdb94bc428b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Timeout</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aac834535a6eab608968bbb44d9773bb1ab55663adb793759edd2082b5194f1cd3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Empty</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aac834535a6eab608968bbb44d9773bb1aaaec822ce6caf3162cbe3cc2cfa1cdc0</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; Context::SharedPtr &gt;</type>
      <name>get_contexts</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>af73bf64bfc1b01f030012cd6d2e6e43c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Client&lt; ServiceT &gt;::SharedPtr</type>
      <name>create_client</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a24538aeccf07c4966c3192623d06440a</anchor>
      <arglist>(std::shared_ptr&lt; node_interfaces::NodeBaseInterface &gt; node_base, std::shared_ptr&lt; node_interfaces::NodeGraphInterface &gt; node_graph, std::shared_ptr&lt; node_interfaces::NodeServicesInterface &gt; node_services, const std::string &amp;service_name, const rmw_qos_profile_t &amp;qos_profile, rclcpp::CallbackGroup::SharedPtr group)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; PublisherT &gt;</type>
      <name>create_publisher</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a67010cd2562ccb9702e91904a7fb3e03</anchor>
      <arglist>(NodeT &amp;node, const std::string &amp;topic_name, const rclcpp::QoS &amp;qos, const rclcpp::PublisherOptionsWithAllocator&lt; AllocatorT &gt; &amp;options=(rclcpp::PublisherOptionsWithAllocator&lt; AllocatorT &gt;()))</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Service&lt; ServiceT &gt;::SharedPtr</type>
      <name>create_service</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a08f170aa649d228312e5eab2497042fb</anchor>
      <arglist>(std::shared_ptr&lt; node_interfaces::NodeBaseInterface &gt; node_base, std::shared_ptr&lt; node_interfaces::NodeServicesInterface &gt; node_services, const std::string &amp;service_name, CallbackT &amp;&amp;callback, const rmw_qos_profile_t &amp;qos_profile, rclcpp::CallbackGroup::SharedPtr group)</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; SubscriptionT &gt;</type>
      <name>create_subscription</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a90a603fbd9fd6cc6ec45e72e575fb182</anchor>
      <arglist>(NodeT &amp;&amp;node, const std::string &amp;topic_name, const rclcpp::QoS &amp;qos, CallbackT &amp;&amp;callback, const rclcpp::SubscriptionOptionsWithAllocator&lt; AllocatorT &gt; &amp;options=(rclcpp::SubscriptionOptionsWithAllocator&lt; AllocatorT &gt;()), typename MessageMemoryStrategyT::SharedPtr msg_mem_strat=(MessageMemoryStrategyT::create_default()))</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::TimerBase::SharedPtr</type>
      <name>create_timer</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a0c53f2add6632d67becd1885d26362c5</anchor>
      <arglist>(std::shared_ptr&lt; node_interfaces::NodeBaseInterface &gt; node_base, std::shared_ptr&lt; node_interfaces::NodeTimersInterface &gt; node_timers, rclcpp::Clock::SharedPtr clock, rclcpp::Duration period, CallbackT &amp;&amp;callback, rclcpp::CallbackGroup::SharedPtr group=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::TimerBase::SharedPtr</type>
      <name>create_timer</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>affb29d356bf00583b0d705b264b3b0ae</anchor>
      <arglist>(NodeT node, rclcpp::Clock::SharedPtr clock, rclcpp::Duration period, CallbackT &amp;&amp;callback, rclcpp::CallbackGroup::SharedPtr group=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::WallTimer&lt; CallbackT &gt;::SharedPtr</type>
      <name>create_wall_timer</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ad987fd245e31c6fd16d70d6ed1186189</anchor>
      <arglist>(std::chrono::duration&lt; DurationRepT, DurationT &gt; period, CallbackT callback, rclcpp::CallbackGroup::SharedPtr group, node_interfaces::NodeBaseInterface *node_base, node_interfaces::NodeTimersInterface *node_timers)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin_some</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ad48c7a9cc4fa34989a0849d708d8f7de</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin_some</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a5e16488d62cc48e5520101f9f4f4102a</anchor>
      <arglist>(rclcpp::Node::SharedPtr node_ptr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a21e13577f5bcc5992de1d7dd08d8652b</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>spin</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a4bb335bc95a6fa8546a6bfcfd087eb57</anchor>
      <arglist>(rclcpp::Node::SharedPtr node_ptr)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::FutureReturnCode</type>
      <name>spin_until_future_complete</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a702eef83e9bdbaabbc89e426668d808c</anchor>
      <arglist>(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, const std::shared_future&lt; FutureT &gt; &amp;future, std::chrono::duration&lt; TimeRepT, TimeT &gt; timeout=std::chrono::duration&lt; TimeRepT, TimeT &gt;(-1))</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::FutureReturnCode</type>
      <name>spin_until_future_complete</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a23b2eb933df4c6b35dc5ffa01b78bcfc</anchor>
      <arglist>(std::shared_ptr&lt; NodeT &gt; node_ptr, const std::shared_future&lt; FutureT &gt; &amp;future, std::chrono::duration&lt; TimeRepT, TimeT &gt; timeout=std::chrono::duration&lt; TimeRepT, TimeT &gt;(-1))</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>expand_topic_or_service_name</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1855b992b1b77f1ed75bab5192aaf2bd</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;node_name, const std::string &amp;namespace_, bool is_service=false)</arglist>
    </member>
    <member kind="function">
      <type>std::ostream &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a9a474b200d67e8a5988daaeb8980fb8b</anchor>
      <arglist>(std::ostream &amp;os, const FutureReturnCode &amp;future_return_code)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a0a11b0ee43bb55e4eca3a79866406e5e</anchor>
      <arglist>(const FutureReturnCode &amp;future_return_code)</arglist>
    </member>
    <member kind="function">
      <type>Logger</type>
      <name>get_logger</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ae7295751947c08312aa69f45fd673171</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>Logger</type>
      <name>get_node_logger</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a32cf150e9157d8ae52206bbb7f1a9310</anchor>
      <arglist>(const rcl_node_t *node)</arglist>
    </member>
    <member kind="function">
      <type>RCLCPP_LOCAL std::string</type>
      <name>extend_name_with_sub_namespace</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a575513c28b9d018ab59102e53f49f57d</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;sub_namespace)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>_to_json_dict_entry</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a4d8f47054b6c550888eca755f3203f05</anchor>
      <arglist>(const Parameter &amp;param)</arglist>
    </member>
    <member kind="function">
      <type>std::ostream &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a6980e39627cfc70b542040ada42ca607</anchor>
      <arglist>(std::ostream &amp;os, const rclcpp::Parameter &amp;pv)</arglist>
    </member>
    <member kind="function">
      <type>std::ostream &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a0e9c5f2bb49142b5f5ad11a8cf1c9328</anchor>
      <arglist>(std::ostream &amp;os, const std::vector&lt; Parameter &gt; &amp;parameters)</arglist>
    </member>
    <member kind="function">
      <type>ParameterMap</type>
      <name>parameter_map_from</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa974dc62646d9123ab206ca602c5e089</anchor>
      <arglist>(const rcl_params_t *const c_params)</arglist>
    </member>
    <member kind="function">
      <type>ParameterValue</type>
      <name>parameter_value_from</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa1a3f3243d1e335570334169979080cd</anchor>
      <arglist>(const rcl_variant_t *const c_value)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a36b93cd43e33e496e2a43f3eb9504b12</anchor>
      <arglist>(ParameterType type)</arglist>
    </member>
    <member kind="function">
      <type>std::ostream &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aed5caea480fefe503c1a7a02b4024dca</anchor>
      <arglist>(std::ostream &amp;os, ParameterType type)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>acc7cfb4c8905865fb02f3c8018657057</anchor>
      <arglist>(const ParameterValue &amp;type)</arglist>
    </member>
    <member kind="function">
      <type>PublisherFactory</type>
      <name>create_publisher_factory</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a16897dedbda576cf6035188bdc7365d7</anchor>
      <arglist>(const rclcpp::PublisherOptionsWithAllocator&lt; AllocatorT &gt; &amp;options)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>qos_policy_name_from_kind</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a7bf3573ad178cad6dcdad0e8cdbdfe6f</anchor>
      <arglist>(rmw_qos_policy_kind_t policy_kind)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa9c7e823d8b8cbde96e0e295065c3836</anchor>
      <arglist>(const QoS &amp;left, const QoS &amp;right)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>af233f254408ae99037eaef31e6655e0d</anchor>
      <arglist>(const QoS &amp;left, const QoS &amp;right)</arglist>
    </member>
    <member kind="function">
      <type>ScopeExit&lt; Callable &gt;</type>
      <name>make_scope_exit</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>afc7846ba2bee783fa9352c7073f30eec</anchor>
      <arglist>(Callable callable)</arglist>
    </member>
    <member kind="function">
      <type>SubscriptionFactory</type>
      <name>create_subscription_factory</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a3de16fd4688e653846d71dbd40f79b09</anchor>
      <arglist>(CallbackT &amp;&amp;callback, const rclcpp::SubscriptionOptionsWithAllocator&lt; AllocatorT &gt; &amp;options, typename MessageMemoryStrategyT::SharedPtr msg_mem_strat, std::shared_ptr&lt; rclcpp::topic_statistics::SubscriptionTopicStatistics&lt; CallbackMessageT &gt;&gt; subscription_topic_stats=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>Time</type>
      <name>operator+</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a31b2ba48a94966a93b36de2e52cae2bb</anchor>
      <arglist>(const rclcpp::Duration &amp;lhs, const rclcpp::Time &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>init</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a2db29afebba8f677bc8660a45bb910bb</anchor>
      <arglist>(int argc, char const *const argv[], const InitOptions &amp;init_options=InitOptions())</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>install_signal_handlers</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a98aab08c64d725e46dee33ee5d705277</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>signal_handlers_installed</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a3944023ee7719c1b3eab2a1cd4e0f3f3</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>uninstall_signal_handlers</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ad74690adaed20915f98b36c0e93d3231</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>init_and_remove_ros_arguments</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ae27196203387b54cc4345b8d1303e45a</anchor>
      <arglist>(int argc, char const *const argv[], const InitOptions &amp;init_options=InitOptions())</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>remove_ros_arguments</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ad4968a767791995c011c57be22cd40c8</anchor>
      <arglist>(int argc, char const *const argv[])</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>ok</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>adbe8ffd2b1769e897f2c50d560812b43</anchor>
      <arglist>(rclcpp::Context::SharedPtr context=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_initialized</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ac9460447dae147a331cab70668a7ddd2</anchor>
      <arglist>(rclcpp::Context::SharedPtr context=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>shutdown</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a493714a679d1591142800416a286689f</anchor>
      <arglist>(rclcpp::Context::SharedPtr context=nullptr, const std::string &amp;reason=&quot;user called rclcpp::shutdown()&quot;)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>on_shutdown</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a01e2c223964ccca7ede393af47fac025</anchor>
      <arglist>(std::function&lt; void()&gt; callback, rclcpp::Context::SharedPtr context=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>sleep_for</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>ac305329e4e97948d4bb216e894caa4ae</anchor>
      <arglist>(const std::chrono::nanoseconds &amp;nanoseconds, rclcpp::Context::SharedPtr context=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>add_will_overflow</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>af238f376176cf3da48adc46b94d29a6a</anchor>
      <arglist>(const T x, const T y)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>add_will_underflow</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a26f62ce86fabd324005231d8d89a8294</anchor>
      <arglist>(const T x, const T y)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>sub_will_overflow</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>aa7c63f4d5146a9054f3b2d8b9ac2070f</anchor>
      <arglist>(const T x, const T y)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>sub_will_underflow</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>af340ddf3a7b82a7a5a4808740a039e69</anchor>
      <arglist>(const T x, const T y)</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_c_string</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a1fdf023cdc167cb3d4d353353cd4cced</anchor>
      <arglist>(const char *string_in)</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>get_c_string</name>
      <anchorfile>namespacerclcpp.html</anchorfile>
      <anchor>a7743e6192ef035115abdeeb90227c45c</anchor>
      <arglist>(const std::string &amp;string_in)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::allocator</name>
    <filename>namespacerclcpp_1_1allocator.html</filename>
    <class kind="class">rclcpp::allocator::AllocatorDeleter</class>
    <member kind="typedef">
      <type>typename std::allocator_traits&lt; Alloc &gt;::template rebind_traits&lt; T &gt;</type>
      <name>AllocRebind</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>a89c84a2945dc1cea1d6dfd4fa72a9dcd</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename std::conditional&lt; std::is_same&lt; typename std::allocator_traits&lt; Alloc &gt;::template rebind_alloc&lt; T &gt;, typename std::allocator&lt; void &gt;::template rebind&lt; T &gt;::other &gt;::value, std::default_delete&lt; T &gt;, AllocatorDeleter&lt; Alloc &gt; &gt;::type</type>
      <name>Deleter</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>a885f50f2cbbab914f65ef687a0edd61b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void *</type>
      <name>retyped_allocate</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>a9e8e54de535dc24500c6b0bd40f66b45</anchor>
      <arglist>(size_t size, void *untyped_allocator)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>retyped_deallocate</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>aaefceff62e574693508452a7a125d9c9</anchor>
      <arglist>(void *untyped_pointer, void *untyped_allocator)</arglist>
    </member>
    <member kind="function">
      <type>void *</type>
      <name>retyped_reallocate</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>a670a5abe8a315e3fdea5e2a1119ad72f</anchor>
      <arglist>(void *untyped_pointer, size_t size, void *untyped_allocator)</arglist>
    </member>
    <member kind="function">
      <type>rcl_allocator_t</type>
      <name>get_rcl_allocator</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>a818a205bf62909c16a2f6439eb4c6a1f</anchor>
      <arglist>(Alloc &amp;allocator)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_allocator_for_deleter</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>af9fd8ff79de9473a17da7526381be91b</anchor>
      <arglist>(D *deleter, Alloc *alloc)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_allocator_for_deleter</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>a4aaee2843f9b3aff54aa8ec555653431</anchor>
      <arglist>(std::default_delete&lt; T &gt; *deleter, std::allocator&lt; U &gt; *alloc)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_allocator_for_deleter</name>
      <anchorfile>namespacerclcpp_1_1allocator.html</anchorfile>
      <anchor>aaaecd19630abe2213cdcf4a3e1a65790</anchor>
      <arglist>(AllocatorDeleter&lt; T &gt; *deleter, Alloc *alloc)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::callback_group</name>
    <filename>namespacerclcpp_1_1callback__group.html</filename>
    <member kind="typedef">
      <type>CallbackGroupType</type>
      <name>CallbackGroupType</name>
      <anchorfile>namespacerclcpp_1_1callback__group.html</anchorfile>
      <anchor>a57d7989f0fc39e917df95b929fef8d0c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>CallbackGroup</type>
      <name>CallbackGroup</name>
      <anchorfile>namespacerclcpp_1_1callback__group.html</anchorfile>
      <anchor>a88b076618efae1cda12782eba9513d63</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::contexts</name>
    <filename>namespacerclcpp_1_1contexts.html</filename>
    <namespace>rclcpp::contexts::default_context</namespace>
    <class kind="class">rclcpp::contexts::DefaultContext</class>
    <member kind="function">
      <type>DefaultContext::SharedPtr</type>
      <name>get_global_default_context</name>
      <anchorfile>namespacerclcpp_1_1contexts.html</anchorfile>
      <anchor>a18a2108b667f7e582db2c9ef216870a6</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::contexts::default_context</name>
    <filename>namespacerclcpp_1_1contexts_1_1default__context.html</filename>
    <member kind="typedef">
      <type>DefaultContext</type>
      <name>DefaultContext</name>
      <anchorfile>namespacerclcpp_1_1contexts_1_1default__context.html</anchorfile>
      <anchor>a4bb405cf2d9eac16028434a4e22dc3f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>DefaultContext::SharedPtr</type>
      <name>get_global_default_context</name>
      <anchorfile>namespacerclcpp_1_1contexts_1_1default__context.html</anchorfile>
      <anchor>af42b47432f5c680a63caab656e935fbb</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::detail</name>
    <filename>namespacerclcpp_1_1detail.html</filename>
    <class kind="class">rclcpp::detail::MutexTwoPriorities</class>
    <class kind="class">rclcpp::detail::RMWImplementationSpecificPayload</class>
    <class kind="class">rclcpp::detail::RMWImplementationSpecificPublisherPayload</class>
    <class kind="class">rclcpp::detail::RMWImplementationSpecificSubscriptionPayload</class>
    <member kind="function">
      <type>bool</type>
      <name>resolve_enable_topic_statistics</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>ab4c905b03a1caa939ff486d085c199e1</anchor>
      <arglist>(const OptionsT &amp;options, const NodeBaseT &amp;node_base)</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::IntraProcessBufferType</type>
      <name>resolve_intra_process_buffer_type</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>a6301cf54892f0767beb71bab3ad190d3</anchor>
      <arglist>(const rclcpp::IntraProcessBufferType buffer_type, const rclcpp::AnySubscriptionCallback&lt; CallbackMessageT, AllocatorT &gt; &amp;any_subscription_callback)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>resolve_use_intra_process</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>ab4e9664da042c42723eba5fbf7c3f998</anchor>
      <arglist>(const OptionsT &amp;options, const NodeBaseT &amp;node_base)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>get_unparsed_ros_arguments</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>a7cff1393aeeab323717e22fa15236da2</anchor>
      <arglist>(int argc, char const *const argv[], rcl_arguments_t *arguments, rcl_allocator_t allocator)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>get_value_helper</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>ae4f35ac4285bf465bb224eacbe09d2dc</anchor>
      <arglist>(const rclcpp::Parameter *parameter)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>get_value_helper&lt; rclcpp::ParameterValue &gt;</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>a014059cadd00427bd36426b2a768566e</anchor>
      <arglist>(const rclcpp::Parameter *parameter)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>get_value_helper&lt; rclcpp::Parameter &gt;</name>
      <anchorfile>namespacerclcpp_1_1detail.html</anchorfile>
      <anchor>af30f36a40ef365a465b6efc25e9eb7cf</anchor>
      <arglist>(const rclcpp::Parameter *parameter)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::exceptions</name>
    <filename>namespacerclcpp_1_1exceptions.html</filename>
    <class kind="class">rclcpp::exceptions::EventNotRegisteredError</class>
    <class kind="class">rclcpp::exceptions::InvalidEventError</class>
    <class kind="class">rclcpp::exceptions::InvalidNamespaceError</class>
    <class kind="class">rclcpp::exceptions::InvalidNodeError</class>
    <class kind="class">rclcpp::exceptions::InvalidNodeNameError</class>
    <class kind="class">rclcpp::exceptions::InvalidParametersException</class>
    <class kind="class">rclcpp::exceptions::InvalidParameterTypeException</class>
    <class kind="class">rclcpp::exceptions::InvalidParameterValueException</class>
    <class kind="class">rclcpp::exceptions::InvalidServiceNameError</class>
    <class kind="class">rclcpp::exceptions::InvalidTopicNameError</class>
    <class kind="class">rclcpp::exceptions::NameValidationError</class>
    <class kind="class">rclcpp::exceptions::ParameterAlreadyDeclaredException</class>
    <class kind="class">rclcpp::exceptions::ParameterImmutableException</class>
    <class kind="class">rclcpp::exceptions::ParameterModifiedInCallbackException</class>
    <class kind="class">rclcpp::exceptions::ParameterNotDeclaredException</class>
    <class kind="class">rclcpp::exceptions::RCLBadAlloc</class>
    <class kind="class">rclcpp::exceptions::RCLError</class>
    <class kind="class">rclcpp::exceptions::RCLErrorBase</class>
    <class kind="class">rclcpp::exceptions::RCLInvalidArgument</class>
    <class kind="class">rclcpp::exceptions::RCLInvalidROSArgsError</class>
    <class kind="class">rclcpp::exceptions::UnimplementedError</class>
    <class kind="class">rclcpp::exceptions::UnknownROSArgsError</class>
    <member kind="function">
      <type>void</type>
      <name>throw_from_rcl_error</name>
      <anchorfile>namespacerclcpp_1_1exceptions.html</anchorfile>
      <anchor>ad1d5563cfff9273f531a82c1dde13516</anchor>
      <arglist>(rcl_ret_t ret, const std::string &amp;prefix=&quot;&quot;, const rcl_error_state_t *error_state=nullptr, void(*reset_error)()=rcl_reset_error)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::executor</name>
    <filename>namespacerclcpp_1_1executor.html</filename>
    <member kind="typedef">
      <type>AnyExecutable</type>
      <name>AnyExecutable</name>
      <anchorfile>namespacerclcpp_1_1executor.html</anchorfile>
      <anchor>ab79f55088380403dbd5e042248e74ce8</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>rclcpp::Executor</type>
      <name>Executor</name>
      <anchorfile>namespacerclcpp_1_1executor.html</anchorfile>
      <anchor>aa7ab7535de7e4a9f4c9dc9ca3333ede3</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>ExecutorOptions</type>
      <name>ExecutorArgs</name>
      <anchorfile>namespacerclcpp_1_1executor.html</anchorfile>
      <anchor>a35debac90796a7551a850e56ad96206e</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>FutureReturnCode</type>
      <name>FutureReturnCode</name>
      <anchorfile>namespacerclcpp_1_1executor.html</anchorfile>
      <anchor>a3b8f8bd56c33a1c2f0ab3d6fe5d4e089</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>rclcpp::ExecutorOptions</type>
      <name>create_default_executor_arguments</name>
      <anchorfile>namespacerclcpp_1_1executor.html</anchorfile>
      <anchor>adfc3aa74cdc0b03340727f779cb58eb9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>namespacerclcpp_1_1executor.html</anchorfile>
      <anchor>a402969c33929fa7d75f95f89514a4df9</anchor>
      <arglist>(const rclcpp::FutureReturnCode &amp;future_return_code)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::executors</name>
    <filename>namespacerclcpp_1_1executors.html</filename>
    <class kind="class">rclcpp::executors::MultiThreadedExecutor</class>
    <class kind="class">rclcpp::executors::SingleThreadedExecutor</class>
    <class kind="class">rclcpp::executors::StaticExecutorEntitiesCollector</class>
    <class kind="class">rclcpp::executors::StaticSingleThreadedExecutor</class>
    <member kind="function">
      <type>rclcpp::FutureReturnCode</type>
      <name>spin_node_until_future_complete</name>
      <anchorfile>namespacerclcpp_1_1executors.html</anchorfile>
      <anchor>a1c80eee179a16b9d564ddb5dba55c625</anchor>
      <arglist>(rclcpp::Executor &amp;executor, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, const std::shared_future&lt; ResponseT &gt; &amp;future, std::chrono::duration&lt; TimeRepT, TimeT &gt; timeout=std::chrono::duration&lt; TimeRepT, TimeT &gt;(-1))</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::FutureReturnCode</type>
      <name>spin_node_until_future_complete</name>
      <anchorfile>namespacerclcpp_1_1executors.html</anchorfile>
      <anchor>a945cba4bc9c68284248b2bd91d168c42</anchor>
      <arglist>(rclcpp::Executor &amp;executor, std::shared_ptr&lt; NodeT &gt; node_ptr, const std::shared_future&lt; ResponseT &gt; &amp;future, std::chrono::duration&lt; TimeRepT, TimeT &gt; timeout=std::chrono::duration&lt; TimeRepT, TimeT &gt;(-1))</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::experimental</name>
    <filename>namespacerclcpp_1_1experimental.html</filename>
    <namespace>rclcpp::experimental::buffers</namespace>
    <class kind="class">rclcpp::experimental::ExecutableList</class>
    <class kind="class">rclcpp::experimental::IntraProcessManager</class>
    <class kind="class">rclcpp::experimental::SubscriptionIntraProcess</class>
    <class kind="class">rclcpp::experimental::SubscriptionIntraProcessBase</class>
    <member kind="function">
      <type>rclcpp::experimental::buffers::IntraProcessBuffer&lt; MessageT, Alloc, Deleter &gt;::UniquePtr</type>
      <name>create_intra_process_buffer</name>
      <anchorfile>namespacerclcpp_1_1experimental.html</anchorfile>
      <anchor>af620d29730460d7f82e51157e1dff688</anchor>
      <arglist>(IntraProcessBufferType buffer_type, rmw_qos_profile_t qos, std::shared_ptr&lt; Alloc &gt; allocator)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::experimental::buffers</name>
    <filename>namespacerclcpp_1_1experimental_1_1buffers.html</filename>
    <class kind="class">rclcpp::experimental::buffers::BufferImplementationBase</class>
    <class kind="class">rclcpp::experimental::buffers::IntraProcessBuffer</class>
    <class kind="class">rclcpp::experimental::buffers::IntraProcessBufferBase</class>
    <class kind="class">rclcpp::experimental::buffers::RingBufferImplementation</class>
    <class kind="class">rclcpp::experimental::buffers::TypedIntraProcessBuffer</class>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::function_traits</name>
    <filename>namespacerclcpp_1_1function__traits.html</filename>
    <class kind="struct">rclcpp::function_traits::arity_comparator</class>
    <class kind="struct">rclcpp::function_traits::check_arguments</class>
    <class kind="struct">rclcpp::function_traits::function_traits</class>
    <class kind="struct">rclcpp::function_traits::function_traits&lt; FunctionT &amp; &gt;</class>
    <class kind="struct">rclcpp::function_traits::function_traits&lt; FunctionT &amp;&amp; &gt;</class>
    <class kind="struct">rclcpp::function_traits::function_traits&lt; ReturnTypeT(*)(Args ...)&gt;</class>
    <class kind="struct">rclcpp::function_traits::function_traits&lt; ReturnTypeT(Args ...)&gt;</class>
    <class kind="struct">rclcpp::function_traits::function_traits&lt; ReturnTypeT(ClassT::*)(Args ...) const &gt;</class>
    <class kind="struct">rclcpp::function_traits::same_arguments</class>
    <class kind="struct">rclcpp::function_traits::tuple_tail</class>
    <class kind="struct">rclcpp::function_traits::tuple_tail&lt; std::tuple&lt; Head, Tail ... &gt; &gt;</class>
    <member kind="variable">
      <type></type>
      <name>__pad0__</name>
      <anchorfile>namespacerclcpp_1_1function__traits.html</anchorfile>
      <anchor>ae3374ee3b21cea1d76f0f68d52a43521</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type></type>
      <name>__pad1__</name>
      <anchorfile>namespacerclcpp_1_1function__traits.html</anchorfile>
      <anchor>a40d2b02c8eca43d060b10b461fe58a42</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type></type>
      <name>__pad2__</name>
      <anchorfile>namespacerclcpp_1_1function__traits.html</anchorfile>
      <anchor>a9b406c4834621946f1fb16ad4400eb4e</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::graph_listener</name>
    <filename>namespacerclcpp_1_1graph__listener.html</filename>
    <class kind="class">rclcpp::graph_listener::GraphListener</class>
    <class kind="class">rclcpp::graph_listener::GraphListenerShutdownError</class>
    <class kind="class">rclcpp::graph_listener::NodeAlreadyAddedError</class>
    <class kind="class">rclcpp::graph_listener::NodeNotFoundError</class>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::memory_strategies</name>
    <filename>namespacerclcpp_1_1memory__strategies.html</filename>
    <namespace>rclcpp::memory_strategies::allocator_memory_strategy</namespace>
    <member kind="function">
      <type>memory_strategy::MemoryStrategy::SharedPtr</type>
      <name>create_default_strategy</name>
      <anchorfile>namespacerclcpp_1_1memory__strategies.html</anchorfile>
      <anchor>a4d6a626858dbf7a3cc0936897764bfe9</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::memory_strategies::allocator_memory_strategy</name>
    <filename>namespacerclcpp_1_1memory__strategies_1_1allocator__memory__strategy.html</filename>
    <class kind="class">rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy</class>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::memory_strategy</name>
    <filename>namespacerclcpp_1_1memory__strategy.html</filename>
    <class kind="class">rclcpp::memory_strategy::MemoryStrategy</class>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::message_memory_strategy</name>
    <filename>namespacerclcpp_1_1message__memory__strategy.html</filename>
    <class kind="class">rclcpp::message_memory_strategy::MessageMemoryStrategy</class>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::node_interfaces</name>
    <filename>namespacerclcpp_1_1node__interfaces.html</filename>
    <class kind="class">rclcpp::node_interfaces::NodeBase</class>
    <class kind="class">rclcpp::node_interfaces::NodeBaseInterface</class>
    <class kind="class">rclcpp::node_interfaces::NodeClock</class>
    <class kind="class">rclcpp::node_interfaces::NodeClockInterface</class>
    <class kind="class">rclcpp::node_interfaces::NodeGraph</class>
    <class kind="class">rclcpp::node_interfaces::NodeGraphInterface</class>
    <class kind="class">rclcpp::node_interfaces::NodeLogging</class>
    <class kind="class">rclcpp::node_interfaces::NodeLoggingInterface</class>
    <class kind="class">rclcpp::node_interfaces::NodeParameters</class>
    <class kind="class">rclcpp::node_interfaces::NodeParametersInterface</class>
    <class kind="class">rclcpp::node_interfaces::NodeServices</class>
    <class kind="class">rclcpp::node_interfaces::NodeServicesInterface</class>
    <class kind="class">rclcpp::node_interfaces::NodeTimers</class>
    <class kind="class">rclcpp::node_interfaces::NodeTimersInterface</class>
    <class kind="class">rclcpp::node_interfaces::NodeTimeSource</class>
    <class kind="class">rclcpp::node_interfaces::NodeTimeSourceInterface</class>
    <class kind="class">rclcpp::node_interfaces::NodeTopics</class>
    <class kind="class">rclcpp::node_interfaces::NodeTopicsInterface</class>
    <class kind="class">rclcpp::node_interfaces::NodeWaitables</class>
    <class kind="class">rclcpp::node_interfaces::NodeWaitablesInterface</class>
    <class kind="struct">rclcpp::node_interfaces::OnSetParametersCallbackHandle</class>
    <class kind="struct">rclcpp::node_interfaces::ParameterInfo</class>
    <class kind="class">rclcpp::node_interfaces::ParameterMutationRecursionGuard</class>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::serialization_traits</name>
    <filename>namespacerclcpp_1_1serialization__traits.html</filename>
    <class kind="struct">rclcpp::serialization_traits::is_serialized_message_class</class>
    <class kind="struct">rclcpp::serialization_traits::is_serialized_message_class&lt; SerializedMessage &gt;</class>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::strategies</name>
    <filename>namespacerclcpp_1_1strategies.html</filename>
    <namespace>rclcpp::strategies::message_pool_memory_strategy</namespace>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::strategies::message_pool_memory_strategy</name>
    <filename>namespacerclcpp_1_1strategies_1_1message__pool__memory__strategy.html</filename>
    <class kind="class">rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy</class>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::subscription_traits</name>
    <filename>namespacerclcpp_1_1subscription__traits.html</filename>
    <class kind="struct">rclcpp::subscription_traits::extract_message_type</class>
    <class kind="struct">rclcpp::subscription_traits::extract_message_type&lt; std::shared_ptr&lt; MessageT &gt; &gt;</class>
    <class kind="struct">rclcpp::subscription_traits::extract_message_type&lt; std::unique_ptr&lt; MessageT, Deleter &gt; &gt;</class>
    <class kind="struct">rclcpp::subscription_traits::has_message_type</class>
    <class kind="struct">rclcpp::subscription_traits::is_serialized_callback</class>
    <class kind="struct">rclcpp::subscription_traits::is_serialized_subscription</class>
    <class kind="struct">rclcpp::subscription_traits::is_serialized_subscription_argument</class>
    <class kind="struct">rclcpp::subscription_traits::is_serialized_subscription_argument&lt; SerializedMessage &gt;</class>
    <class kind="struct">rclcpp::subscription_traits::is_serialized_subscription_argument&lt; std::shared_ptr&lt; SerializedMessage &gt; &gt;</class>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::topic_statistics</name>
    <filename>namespacerclcpp_1_1topic__statistics.html</filename>
    <class kind="class">rclcpp::topic_statistics::SubscriptionTopicStatistics</class>
    <member kind="variable">
      <type>constexpr const char</type>
      <name>kDefaultPublishTopicName</name>
      <anchorfile>namespacerclcpp_1_1topic__statistics.html</anchorfile>
      <anchor>a54fcc5dacb1b61b72f17213ea9b347ca</anchor>
      <arglist>[]</arglist>
    </member>
    <member kind="variable">
      <type>constexpr const std::chrono::milliseconds</type>
      <name>kDefaultPublishingPeriod</name>
      <anchorfile>namespacerclcpp_1_1topic__statistics.html</anchorfile>
      <anchor>a6566f900f8440cc00d8870fdb2917c33</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::type_support</name>
    <filename>namespacerclcpp_1_1type__support.html</filename>
    <member kind="function">
      <type>const rosidl_message_type_support_t *</type>
      <name>get_intra_process_message_msg_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>a66bd1e46c8223c97596e8c3def51103e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_message_type_support_t *</type>
      <name>get_parameter_event_msg_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>a3a33f79e0edf74a7a3c7bbb15fac4f30</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_message_type_support_t *</type>
      <name>get_set_parameters_result_msg_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>afe2a99fc964d8f98b0cca073b5aa104d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_message_type_support_t *</type>
      <name>get_parameter_descriptor_msg_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>a1e6abad99cddb01e195ae6cf43da259a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_message_type_support_t *</type>
      <name>get_list_parameters_result_msg_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>ab09e2f3d329ea12b8078cf752b3d15b8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_service_type_support_t *</type>
      <name>get_get_parameters_srv_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>ad46e551c5bb32eebbf49d6395ab2c498</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_service_type_support_t *</type>
      <name>get_get_parameter_types_srv_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>a1a7b50e715313a225925f03bc3808ae4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_service_type_support_t *</type>
      <name>get_set_parameters_srv_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>af043d28ff378d26b0d4e478900bb19d5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_service_type_support_t *</type>
      <name>get_list_parameters_srv_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>af9023d99b556bced4348bb9a09fe194e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_service_type_support_t *</type>
      <name>get_describe_parameters_srv_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>a6576186afeeb70d483a98be8573a911a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const rosidl_service_type_support_t *</type>
      <name>get_set_parameters_atomically_srv_type_support</name>
      <anchorfile>namespacerclcpp_1_1type__support.html</anchorfile>
      <anchor>a2339e1ef611d5baeebf698412bdafaaa</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::wait_set_policies</name>
    <filename>namespacerclcpp_1_1wait__set__policies.html</filename>
    <namespace>rclcpp::wait_set_policies::detail</namespace>
    <class kind="class">rclcpp::wait_set_policies::DynamicStorage</class>
    <class kind="class">rclcpp::wait_set_policies::SequentialSynchronization</class>
    <class kind="class">rclcpp::wait_set_policies::StaticStorage</class>
    <class kind="class">rclcpp::wait_set_policies::ThreadSafeSynchronization</class>
  </compound>
  <compound kind="namespace">
    <name>rclcpp::wait_set_policies::detail</name>
    <filename>namespacerclcpp_1_1wait__set__policies_1_1detail.html</filename>
    <class kind="class">rclcpp::wait_set_policies::detail::StoragePolicyCommon</class>
    <class kind="class">rclcpp::wait_set_policies::detail::SynchronizationPolicyCommon</class>
    <class kind="class">rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock</class>
  </compound>
  <compound kind="namespace">
    <name>std</name>
    <filename>namespacestd.html</filename>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>namespacestd.html</anchorfile>
      <anchor>a1d2324ac3fe483483ba53560318b8639</anchor>
      <arglist>(const rclcpp::Parameter &amp;param)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>namespacestd.html</anchorfile>
      <anchor>ad5ab824f9738a59b56721b0a1517bda9</anchor>
      <arglist>(const std::vector&lt; rclcpp::Parameter &gt; &amp;parameters)</arglist>
    </member>
  </compound>
  <compound kind="page">
    <name>md_include_rclcpp_detail_README</name>
    <title>README</title>
    <filename>md_include_rclcpp_detail_README</filename>
  </compound>
  <compound kind="page">
    <name>md_include_rclcpp_experimental_README</name>
    <title>README</title>
    <filename>md_include_rclcpp_experimental_README</filename>
  </compound>
  <compound kind="page">
    <name>index</name>
    <title>rclcpp: ROS Client Library for C++</title>
    <filename>index</filename>
  </compound>
</tagfile>
