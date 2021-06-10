<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.9.1">
  <compound kind="struct">
    <name>nlohmann::adl_serializer</name>
    <filename>structnlohmann_1_1adl__serializer.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <member kind="function" static="yes">
      <type>static auto</type>
      <name>from_json</name>
      <anchorfile>structnlohmann_1_1adl__serializer_abdae6028cdef1bf1838b47eeb7022ebc.html</anchorfile>
      <anchor>abdae6028cdef1bf1838b47eeb7022ebc</anchor>
      <arglist>(BasicJsonType &amp;&amp;j) noexcept(noexcept(::nlohmann::from_json(std::forward&lt; BasicJsonType &gt;(j), detail::identity_tag&lt; TargetType &gt; {}))) -&gt; decltype(::nlohmann::from_json(std::forward&lt; BasicJsonType &gt;(j), detail::identity_tag&lt; TargetType &gt; {}))</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static auto</type>
      <name>from_json</name>
      <anchorfile>structnlohmann_1_1adl__serializer_a8180f52bf21fc610705bc521f22116ae.html</anchorfile>
      <anchor>a8180f52bf21fc610705bc521f22116ae</anchor>
      <arglist>(BasicJsonType &amp;&amp;j, TargetType &amp;val) noexcept(noexcept(::nlohmann::from_json(std::forward&lt; BasicJsonType &gt;(j), val))) -&gt; decltype(::nlohmann::from_json(std::forward&lt; BasicJsonType &gt;(j), val), void())</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static auto</type>
      <name>to_json</name>
      <anchorfile>structnlohmann_1_1adl__serializer_afdcc8f0204ce8ac7d28a5403f6e1f0e7.html</anchorfile>
      <anchor>afdcc8f0204ce8ac7d28a5403f6e1f0e7</anchor>
      <arglist>(BasicJsonType &amp;j, TargetType &amp;&amp;val) noexcept(noexcept(::nlohmann::to_json(j, std::forward&lt; TargetType &gt;(val)))) -&gt; decltype(::nlohmann::to_json(j, std::forward&lt; TargetType &gt;(val)), void())</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>nlohmann::basic_json</name>
    <filename>classnlohmann_1_1basic__json.html</filename>
    <templarg>ObjectType</templarg>
    <templarg>ArrayType</templarg>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <templarg>AllocatorType</templarg>
    <templarg>JSONSerializer</templarg>
    <templarg></templarg>
    <member kind="typedef">
      <type>detail::cbor_tag_handler_t</type>
      <name>cbor_tag_handler_t</name>
      <anchorfile>classnlohmann_1_1basic__json_a54951d14f0dd10cc3cfdaa24f8bfd15c.html</anchorfile>
      <anchor>a54951d14f0dd10cc3cfdaa24f8bfd15c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>detail::error_handler_t</type>
      <name>error_handler_t</name>
      <anchorfile>classnlohmann_1_1basic__json_a1e7ca76cc3f62626b380be5e18a002d5.html</anchorfile>
      <anchor>a1e7ca76cc3f62626b380be5e18a002d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::initializer_list&lt; detail::json_ref&lt; basic_json &gt; &gt;</type>
      <name>initializer_list_t</name>
      <anchorfile>classnlohmann_1_1basic__json_ac569f292a070dfd2f6b69c16e746095a.html</anchorfile>
      <anchor>ac569f292a070dfd2f6b69c16e746095a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>detail::input_format_t</type>
      <name>input_format_t</name>
      <anchorfile>classnlohmann_1_1basic__json_a211cf53702226ad9fb3c939a3a3d3689.html</anchorfile>
      <anchor>a211cf53702226ad9fb3c939a3a3d3689</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>::nlohmann::json_pointer&lt; basic_json &gt;</type>
      <name>json_pointer</name>
      <anchorfile>classnlohmann_1_1basic__json_aa8f1f93b32da01b42413643be32b2c27.html</anchorfile>
      <anchor>aa8f1f93b32da01b42413643be32b2c27</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>json_sax&lt; basic_json &gt;</type>
      <name>json_sax_t</name>
      <anchorfile>classnlohmann_1_1basic__json_a164b1094a1a9feb54e400d8510bb0b12.html</anchorfile>
      <anchor>a164b1094a1a9feb54e400d8510bb0b12</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>JSONSerializer&lt; T, SFINAE &gt;</type>
      <name>json_serializer</name>
      <anchorfile>classnlohmann_1_1basic__json_ad6ebc5da7ced975bb184133750e7d49f.html</anchorfile>
      <anchor>ad6ebc5da7ced975bb184133750e7d49f</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>detail::parse_event_t</type>
      <name>parse_event_t</name>
      <anchorfile>classnlohmann_1_1basic__json_a24086b03c5c063849df0307f78c41c54.html</anchorfile>
      <anchor>a24086b03c5c063849df0307f78c41c54</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>detail::parser_callback_t&lt; basic_json &gt;</type>
      <name>parser_callback_t</name>
      <anchorfile>classnlohmann_1_1basic__json_a0273d074462644e5d5a7ff313ad0d742.html</anchorfile>
      <anchor>a0273d074462644e5d5a7ff313ad0d742</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>detail::value_t</type>
      <name>value_t</name>
      <anchorfile>classnlohmann_1_1basic__json_ac68cb65a7f3517f0c5b1d3a4967406ad.html</anchorfile>
      <anchor>ac68cb65a7f3517f0c5b1d3a4967406ad</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>JSON_HEDLEY_RETURNS_NON_NULL const char *</type>
      <name>type_name</name>
      <anchorfile>classnlohmann_1_1basic__json_a459dbfcd47bd632ca82ca8ff8db278c8.html</anchorfile>
      <anchor>a459dbfcd47bd632ca82ca8ff8db278c8</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static allocator_type</type>
      <name>get_allocator</name>
      <anchorfile>classnlohmann_1_1basic__json_afc55e7dca1a243b0d5011564824c0267.html</anchorfile>
      <anchor>afc55e7dca1a243b0d5011564824c0267</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>meta</name>
      <anchorfile>classnlohmann_1_1basic__json_a351b4f65014f6c2b8b2832847d44bbd7.html</anchorfile>
      <anchor>a351b4f65014f6c2b8b2832847d44bbd7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable">
      <type>JSON_PRIVATE_UNLESS_TESTED</type>
      <name>__pad3__</name>
      <anchorfile>classnlohmann_1_1basic__json_aa52fb28bbfe1d5484808cad53c35d76c.html</anchorfile>
      <anchor>aa52fb28bbfe1d5484808cad53c35d76c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>json_value</type>
      <name>m_value</name>
      <anchorfile>classnlohmann_1_1basic__json_a72f1c0ede41f166429ce3fe7c2ffefc0.html</anchorfile>
      <anchor>a72f1c0ede41f166429ce3fe7c2ffefc0</anchor>
      <arglist></arglist>
    </member>
    <member kind="friend">
      <type>friend struct</type>
      <name>detail::external_constructor</name>
      <anchorfile>classnlohmann_1_1basic__json_a6275ed57bae6866cdf5db5370a7ad47c.html</anchorfile>
      <anchor>a6275ed57bae6866cdf5db5370a7ad47c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>detail::exception</type>
      <name>exception</name>
      <anchorfile>classnlohmann_1_1basic__json_a14824c27188d2fee4861806cd5f23d22.html</anchorfile>
      <anchor>a14824c27188d2fee4861806cd5f23d22</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>detail::parse_error</type>
      <name>parse_error</name>
      <anchorfile>classnlohmann_1_1basic__json_a555b05e9da63d486126759922685a37a.html</anchorfile>
      <anchor>a555b05e9da63d486126759922685a37a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>detail::invalid_iterator</type>
      <name>invalid_iterator</name>
      <anchorfile>classnlohmann_1_1basic__json_a6ccc9788413fd58de998fe92743cb4aa.html</anchorfile>
      <anchor>a6ccc9788413fd58de998fe92743cb4aa</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>detail::type_error</type>
      <name>type_error</name>
      <anchorfile>classnlohmann_1_1basic__json_ace5bf851eafe85bd6332f978991bc11c.html</anchorfile>
      <anchor>ace5bf851eafe85bd6332f978991bc11c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>detail::out_of_range</type>
      <name>out_of_range</name>
      <anchorfile>classnlohmann_1_1basic__json_a2251d8523fa6d16c0fba6388ffa2ef8c.html</anchorfile>
      <anchor>a2251d8523fa6d16c0fba6388ffa2ef8c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>detail::other_error</type>
      <name>other_error</name>
      <anchorfile>classnlohmann_1_1basic__json_a6fc373c99facc37aadbc5651b3d6631d.html</anchorfile>
      <anchor>a6fc373c99facc37aadbc5651b3d6631d</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>basic_json</type>
      <name>value_type</name>
      <anchorfile>classnlohmann_1_1basic__json_a57c816a20c1d3ccc9bbc2972829da847.html</anchorfile>
      <anchor>a57c816a20c1d3ccc9bbc2972829da847</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>value_type &amp;</type>
      <name>reference</name>
      <anchorfile>classnlohmann_1_1basic__json_a220ae98554a76205fb7f8822d36b2d5a.html</anchorfile>
      <anchor>a220ae98554a76205fb7f8822d36b2d5a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>const value_type &amp;</type>
      <name>const_reference</name>
      <anchorfile>classnlohmann_1_1basic__json_ab8a1c33ee7b154fc41ca2545aa9724e6.html</anchorfile>
      <anchor>ab8a1c33ee7b154fc41ca2545aa9724e6</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::ptrdiff_t</type>
      <name>difference_type</name>
      <anchorfile>classnlohmann_1_1basic__json_a3d20d11e5dfe95084a76f62eca54fadd.html</anchorfile>
      <anchor>a3d20d11e5dfe95084a76f62eca54fadd</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::size_t</type>
      <name>size_type</name>
      <anchorfile>classnlohmann_1_1basic__json_a3ada29bca70b4965f6fd37ec1c8f85f7.html</anchorfile>
      <anchor>a3ada29bca70b4965f6fd37ec1c8f85f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>AllocatorType&lt; basic_json &gt;</type>
      <name>allocator_type</name>
      <anchorfile>classnlohmann_1_1basic__json_ad38ae80f1e99d4b1f33c99fea4611457.html</anchorfile>
      <anchor>ad38ae80f1e99d4b1f33c99fea4611457</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename std::allocator_traits&lt; allocator_type &gt;::pointer</type>
      <name>pointer</name>
      <anchorfile>classnlohmann_1_1basic__json_a42e5c23402f4c2e1df487e1d102bc5fa.html</anchorfile>
      <anchor>a42e5c23402f4c2e1df487e1d102bc5fa</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename std::allocator_traits&lt; allocator_type &gt;::const_pointer</type>
      <name>const_pointer</name>
      <anchorfile>classnlohmann_1_1basic__json_a4108c5148f1d7cf13c2681e22f141a10.html</anchorfile>
      <anchor>a4108c5148f1d7cf13c2681e22f141a10</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>iter_impl&lt; basic_json &gt;</type>
      <name>iterator</name>
      <anchorfile>classnlohmann_1_1basic__json_aa549b2b382916b3baafb526e5cb410bd.html</anchorfile>
      <anchor>aa549b2b382916b3baafb526e5cb410bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>iter_impl&lt; const basic_json &gt;</type>
      <name>const_iterator</name>
      <anchorfile>classnlohmann_1_1basic__json_aebd2cfa7e4ded4e97cde9269bfeeea38.html</anchorfile>
      <anchor>aebd2cfa7e4ded4e97cde9269bfeeea38</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>json_reverse_iterator&lt; typename basic_json::iterator &gt;</type>
      <name>reverse_iterator</name>
      <anchorfile>classnlohmann_1_1basic__json_a5b8c0ebedd920b507f4f7ff4e19bf3c6.html</anchorfile>
      <anchor>a5b8c0ebedd920b507f4f7ff4e19bf3c6</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>json_reverse_iterator&lt; typename basic_json::const_iterator &gt;</type>
      <name>const_reverse_iterator</name>
      <anchorfile>classnlohmann_1_1basic__json_aa7dba16ed9ee97380aeb17a207dd919a.html</anchorfile>
      <anchor>aa7dba16ed9ee97380aeb17a207dd919a</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::less&lt; StringType &gt;</type>
      <name>object_comparator_t</name>
      <anchorfile>classnlohmann_1_1basic__json_ac26c2e8d6bcaccde372ceedd81851200.html</anchorfile>
      <anchor>ac26c2e8d6bcaccde372ceedd81851200</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>ObjectType&lt; StringType, basic_json, object_comparator_t, AllocatorType&lt; std::pair&lt; const StringType, basic_json &gt; &gt;&gt;</type>
      <name>object_t</name>
      <anchorfile>classnlohmann_1_1basic__json_aef3ff5a73597850597d1d40db9edd376.html</anchorfile>
      <anchor>aef3ff5a73597850597d1d40db9edd376</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>ArrayType&lt; basic_json, AllocatorType&lt; basic_json &gt; &gt;</type>
      <name>array_t</name>
      <anchorfile>classnlohmann_1_1basic__json_a858c1cf8407bc06494e3a1114a3b73e7.html</anchorfile>
      <anchor>a858c1cf8407bc06494e3a1114a3b73e7</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>StringType</type>
      <name>string_t</name>
      <anchorfile>classnlohmann_1_1basic__json_a33593865ffb1860323dcbd52425b90c8.html</anchorfile>
      <anchor>a33593865ffb1860323dcbd52425b90c8</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>BooleanType</type>
      <name>boolean_t</name>
      <anchorfile>classnlohmann_1_1basic__json_a44fd1a12c9c54623c942b430e7a72937.html</anchorfile>
      <anchor>a44fd1a12c9c54623c942b430e7a72937</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>NumberIntegerType</type>
      <name>number_integer_t</name>
      <anchorfile>classnlohmann_1_1basic__json_a11e390944da90db83089eb2426a749d3.html</anchorfile>
      <anchor>a11e390944da90db83089eb2426a749d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>NumberUnsignedType</type>
      <name>number_unsigned_t</name>
      <anchorfile>classnlohmann_1_1basic__json_ae09af9c23351b7245d9be4d1b2035fef.html</anchorfile>
      <anchor>ae09af9c23351b7245d9be4d1b2035fef</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>NumberFloatType</type>
      <name>number_float_t</name>
      <anchorfile>classnlohmann_1_1basic__json_a5b8abaebd922d82d69756327c0c347e6.html</anchorfile>
      <anchor>a5b8abaebd922d82d69756327c0c347e6</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>nlohmann::byte_container_with_subtype&lt; BinaryType &gt;</type>
      <name>binary_t</name>
      <anchorfile>classnlohmann_1_1basic__json_ad6c955145bebde84d93991ffed7cd389.html</anchorfile>
      <anchor>ad6c955145bebde84d93991ffed7cd389</anchor>
      <arglist></arglist>
    </member>
    <member kind="friend">
      <type>friend void</type>
      <name>swap</name>
      <anchorfile>classnlohmann_1_1basic__json_aee0ae36cbfb0336832ebc0374c3c7679.html</anchorfile>
      <anchor>aee0ae36cbfb0336832ebc0374c3c7679</anchor>
      <arglist>(reference left, reference right) noexcept(std::is_nothrow_move_constructible&lt; value_t &gt;::value &amp;&amp;std::is_nothrow_move_assignable&lt; value_t &gt;::value &amp;&amp;std::is_nothrow_move_constructible&lt; json_value &gt;::value &amp;&amp;std::is_nothrow_move_assignable&lt; json_value &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>clear</name>
      <anchorfile>classnlohmann_1_1basic__json_a946cc8f30d8b1d6609b57387b647fe53.html</anchorfile>
      <anchor>a946cc8f30d8b1d6609b57387b647fe53</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>push_back</name>
      <anchorfile>classnlohmann_1_1basic__json_ab9e0253c92736db021840105d374c4c4.html</anchorfile>
      <anchor>ab9e0253c92736db021840105d374c4c4</anchor>
      <arglist>(basic_json &amp;&amp;val)</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>operator+=</name>
      <anchorfile>classnlohmann_1_1basic__json_a40226d9c84fcb9cb948ae0c27b842c57.html</anchorfile>
      <anchor>a40226d9c84fcb9cb948ae0c27b842c57</anchor>
      <arglist>(basic_json &amp;&amp;val)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>push_back</name>
      <anchorfile>classnlohmann_1_1basic__json_a3405d38087e13994a5a4556065b0be6d.html</anchorfile>
      <anchor>a3405d38087e13994a5a4556065b0be6d</anchor>
      <arglist>(const basic_json &amp;val)</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>operator+=</name>
      <anchorfile>classnlohmann_1_1basic__json_af643a4baa91f484b11af0e4437183115.html</anchorfile>
      <anchor>af643a4baa91f484b11af0e4437183115</anchor>
      <arglist>(const basic_json &amp;val)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>push_back</name>
      <anchorfile>classnlohmann_1_1basic__json_ad704839e6a5195e3b76f22e2b9aa63ee.html</anchorfile>
      <anchor>ad704839e6a5195e3b76f22e2b9aa63ee</anchor>
      <arglist>(const typename object_t::value_type &amp;val)</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>operator+=</name>
      <anchorfile>classnlohmann_1_1basic__json_ae300819781bce2193369609457f70f30.html</anchorfile>
      <anchor>ae300819781bce2193369609457f70f30</anchor>
      <arglist>(const typename object_t::value_type &amp;val)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>push_back</name>
      <anchorfile>classnlohmann_1_1basic__json_a4567cf75f19b1efca090f75d7a8a350a.html</anchorfile>
      <anchor>a4567cf75f19b1efca090f75d7a8a350a</anchor>
      <arglist>(initializer_list_t init)</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>operator+=</name>
      <anchorfile>classnlohmann_1_1basic__json_ac48302a8b4f8c2a6e30c2a7bff6abc49.html</anchorfile>
      <anchor>ac48302a8b4f8c2a6e30c2a7bff6abc49</anchor>
      <arglist>(initializer_list_t init)</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>emplace_back</name>
      <anchorfile>classnlohmann_1_1basic__json_a15c0a5db4fb12d49433801bbe6436bfb.html</anchorfile>
      <anchor>a15c0a5db4fb12d49433801bbe6436bfb</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; iterator, bool &gt;</type>
      <name>emplace</name>
      <anchorfile>classnlohmann_1_1basic__json_ac479e609cbd03948bd3e85fb441b66e5.html</anchorfile>
      <anchor>ac479e609cbd03948bd3e85fb441b66e5</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>iterator</type>
      <name>insert_iterator</name>
      <anchorfile>classnlohmann_1_1basic__json_ab5c8034e997c5b852b92bdc6a3f70994.html</anchorfile>
      <anchor>ab5c8034e997c5b852b92bdc6a3f70994</anchor>
      <arglist>(const_iterator pos, Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>iterator</type>
      <name>insert</name>
      <anchorfile>classnlohmann_1_1basic__json_aeb86e8478e20d95970a8b61ff01dce3b.html</anchorfile>
      <anchor>aeb86e8478e20d95970a8b61ff01dce3b</anchor>
      <arglist>(const_iterator pos, const basic_json &amp;val)</arglist>
    </member>
    <member kind="function">
      <type>iterator</type>
      <name>insert</name>
      <anchorfile>classnlohmann_1_1basic__json_a9c5b9de8a4a759861cb600b38a6c81b1.html</anchorfile>
      <anchor>a9c5b9de8a4a759861cb600b38a6c81b1</anchor>
      <arglist>(const_iterator pos, basic_json &amp;&amp;val)</arglist>
    </member>
    <member kind="function">
      <type>iterator</type>
      <name>insert</name>
      <anchorfile>classnlohmann_1_1basic__json_a71e197e6cc78c3960011f68a75f8ef22.html</anchorfile>
      <anchor>a71e197e6cc78c3960011f68a75f8ef22</anchor>
      <arglist>(const_iterator pos, size_type cnt, const basic_json &amp;val)</arglist>
    </member>
    <member kind="function">
      <type>iterator</type>
      <name>insert</name>
      <anchorfile>classnlohmann_1_1basic__json_a8137d5471edcd71606e42155ed9c23e2.html</anchorfile>
      <anchor>a8137d5471edcd71606e42155ed9c23e2</anchor>
      <arglist>(const_iterator pos, const_iterator first, const_iterator last)</arglist>
    </member>
    <member kind="function">
      <type>iterator</type>
      <name>insert</name>
      <anchorfile>classnlohmann_1_1basic__json_a856b8764efd21dac4205a00fec82e09a.html</anchorfile>
      <anchor>a856b8764efd21dac4205a00fec82e09a</anchor>
      <arglist>(const_iterator pos, initializer_list_t ilist)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>insert</name>
      <anchorfile>classnlohmann_1_1basic__json_a0181d03c6314bedcbad2e92d3676223c.html</anchorfile>
      <anchor>a0181d03c6314bedcbad2e92d3676223c</anchor>
      <arglist>(const_iterator first, const_iterator last)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>update</name>
      <anchorfile>classnlohmann_1_1basic__json_a377819905d567f6f523dcbc592cb6356.html</anchorfile>
      <anchor>a377819905d567f6f523dcbc592cb6356</anchor>
      <arglist>(const_reference j)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>update</name>
      <anchorfile>classnlohmann_1_1basic__json_a9f9e5f668474280acc9bd7f5410b9392.html</anchorfile>
      <anchor>a9f9e5f668474280acc9bd7f5410b9392</anchor>
      <arglist>(const_iterator first, const_iterator last)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>swap</name>
      <anchorfile>classnlohmann_1_1basic__json_a94295a06e0e7b3867fe83afbee4cb202.html</anchorfile>
      <anchor>a94295a06e0e7b3867fe83afbee4cb202</anchor>
      <arglist>(reference other) noexcept(std::is_nothrow_move_constructible&lt; value_t &gt;::value &amp;&amp;std::is_nothrow_move_assignable&lt; value_t &gt;::value &amp;&amp;std::is_nothrow_move_constructible&lt; json_value &gt;::value &amp;&amp;std::is_nothrow_move_assignable&lt; json_value &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>swap</name>
      <anchorfile>classnlohmann_1_1basic__json_a76126242de262f6d38cadda19e0d13e1.html</anchorfile>
      <anchor>a76126242de262f6d38cadda19e0d13e1</anchor>
      <arglist>(array_t &amp;other)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>swap</name>
      <anchorfile>classnlohmann_1_1basic__json_a57b86bdcfc55557dacc36969adb0417e.html</anchorfile>
      <anchor>a57b86bdcfc55557dacc36969adb0417e</anchor>
      <arglist>(object_t &amp;other)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>swap</name>
      <anchorfile>classnlohmann_1_1basic__json_aac916df9561daf4eaf2372119fe91899.html</anchorfile>
      <anchor>aac916df9561daf4eaf2372119fe91899</anchor>
      <arglist>(string_t &amp;other)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>swap</name>
      <anchorfile>classnlohmann_1_1basic__json_aa242e339ebc7583e114f2167a83f8c90.html</anchorfile>
      <anchor>aa242e339ebc7583e114f2167a83f8c90</anchor>
      <arglist>(binary_t &amp;other)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>swap</name>
      <anchorfile>classnlohmann_1_1basic__json_a749a1f5091a5e63ccfe919e0aef986af.html</anchorfile>
      <anchor>a749a1f5091a5e63ccfe919e0aef986af</anchor>
      <arglist>(typename binary_t::container_type &amp;other)</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator==</name>
      <anchorfile>classnlohmann_1_1basic__json_a122640e7e2db1814fc7bbb3c122ec76e.html</anchorfile>
      <anchor>a122640e7e2db1814fc7bbb3c122ec76e</anchor>
      <arglist>(const_reference lhs, const_reference rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator==</name>
      <anchorfile>classnlohmann_1_1basic__json_a107a085c92ec4e062d1185b2d09c7978.html</anchorfile>
      <anchor>a107a085c92ec4e062d1185b2d09c7978</anchor>
      <arglist>(const_reference lhs, ScalarType rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator==</name>
      <anchorfile>classnlohmann_1_1basic__json_a24ae7acd5b06ae49cfe1c94633436b68.html</anchorfile>
      <anchor>a24ae7acd5b06ae49cfe1c94633436b68</anchor>
      <arglist>(ScalarType lhs, const_reference rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator!=</name>
      <anchorfile>classnlohmann_1_1basic__json_a6e2e21da48f5d9471716cd868a068327.html</anchorfile>
      <anchor>a6e2e21da48f5d9471716cd868a068327</anchor>
      <arglist>(const_reference lhs, const_reference rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator!=</name>
      <anchorfile>classnlohmann_1_1basic__json_a4d581dc2ab78091885f9af24d13749dc.html</anchorfile>
      <anchor>a4d581dc2ab78091885f9af24d13749dc</anchor>
      <arglist>(const_reference lhs, ScalarType rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator!=</name>
      <anchorfile>classnlohmann_1_1basic__json_aa9861979059799375c0fff68174610ba.html</anchorfile>
      <anchor>aa9861979059799375c0fff68174610ba</anchor>
      <arglist>(ScalarType lhs, const_reference rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator&lt;</name>
      <anchorfile>classnlohmann_1_1basic__json_aacd442b66140c764c594ac8ad7dfd5b3.html</anchorfile>
      <anchor>aacd442b66140c764c594ac8ad7dfd5b3</anchor>
      <arglist>(const_reference lhs, const_reference rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator&lt;</name>
      <anchorfile>classnlohmann_1_1basic__json_a52907b78829a55473f2548530cf2c1c7.html</anchorfile>
      <anchor>a52907b78829a55473f2548530cf2c1c7</anchor>
      <arglist>(const_reference lhs, ScalarType rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator&lt;</name>
      <anchorfile>classnlohmann_1_1basic__json_a71f65ff3abee4c140e27ca64fa327973.html</anchorfile>
      <anchor>a71f65ff3abee4c140e27ca64fa327973</anchor>
      <arglist>(ScalarType lhs, const_reference rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator&lt;=</name>
      <anchorfile>classnlohmann_1_1basic__json_a5c8bb5200f5eac10d31e26be46e5b1ac.html</anchorfile>
      <anchor>a5c8bb5200f5eac10d31e26be46e5b1ac</anchor>
      <arglist>(const_reference lhs, const_reference rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator&lt;=</name>
      <anchorfile>classnlohmann_1_1basic__json_ae7bce6989e2bf72aa0784ca6755e22a9.html</anchorfile>
      <anchor>ae7bce6989e2bf72aa0784ca6755e22a9</anchor>
      <arglist>(const_reference lhs, ScalarType rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator&lt;=</name>
      <anchorfile>classnlohmann_1_1basic__json_a4e96b98c1f8dfccaaa94aa0d5e77427c.html</anchorfile>
      <anchor>a4e96b98c1f8dfccaaa94aa0d5e77427c</anchor>
      <arglist>(ScalarType lhs, const_reference rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator&gt;</name>
      <anchorfile>classnlohmann_1_1basic__json_a87db51b6b936fb2ea293cdbc8702dcb8.html</anchorfile>
      <anchor>a87db51b6b936fb2ea293cdbc8702dcb8</anchor>
      <arglist>(const_reference lhs, const_reference rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator&gt;</name>
      <anchorfile>classnlohmann_1_1basic__json_a9ea8e0e86820fcb3ab0fc937d41226e7.html</anchorfile>
      <anchor>a9ea8e0e86820fcb3ab0fc937d41226e7</anchor>
      <arglist>(const_reference lhs, ScalarType rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator&gt;</name>
      <anchorfile>classnlohmann_1_1basic__json_a67771044f08fd07105b34667615d9e0e.html</anchorfile>
      <anchor>a67771044f08fd07105b34667615d9e0e</anchor>
      <arglist>(ScalarType lhs, const_reference rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator&gt;=</name>
      <anchorfile>classnlohmann_1_1basic__json_a74a943800c7f103d0990d7eef82c6453.html</anchorfile>
      <anchor>a74a943800c7f103d0990d7eef82c6453</anchor>
      <arglist>(const_reference lhs, const_reference rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator&gt;=</name>
      <anchorfile>classnlohmann_1_1basic__json_ab83d11de4db25633f93e067aa218cae9.html</anchorfile>
      <anchor>ab83d11de4db25633f93e067aa218cae9</anchor>
      <arglist>(const_reference lhs, ScalarType rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend bool</type>
      <name>operator&gt;=</name>
      <anchorfile>classnlohmann_1_1basic__json_a94c61e0128f0794e7a34e1aecee5c445.html</anchorfile>
      <anchor>a94c61e0128f0794e7a34e1aecee5c445</anchor>
      <arglist>(ScalarType lhs, const_reference rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend std::ostream &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classnlohmann_1_1basic__json_a5e34c5435e557d0bf666bd7311211405.html</anchorfile>
      <anchor>a5e34c5435e557d0bf666bd7311211405</anchor>
      <arglist>(std::ostream &amp;o, const basic_json &amp;j)</arglist>
    </member>
    <member kind="friend">
      <type>friend std::ostream &amp;</type>
      <name>operator&gt;&gt;</name>
      <anchorfile>classnlohmann_1_1basic__json_a34d6a60dd99e9f33b8273a1c8db5669b.html</anchorfile>
      <anchor>a34d6a60dd99e9f33b8273a1c8db5669b</anchor>
      <arglist>(const basic_json &amp;j, std::ostream &amp;o)</arglist>
    </member>
    <member kind="friend">
      <type>friend std::istream &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classnlohmann_1_1basic__json_a60ca396028b8d9714c6e10efbf475af6.html</anchorfile>
      <anchor>a60ca396028b8d9714c6e10efbf475af6</anchor>
      <arglist>(basic_json &amp;j, std::istream &amp;i)</arglist>
    </member>
    <member kind="friend">
      <type>friend std::istream &amp;</type>
      <name>operator&gt;&gt;</name>
      <anchorfile>classnlohmann_1_1basic__json_aaf363408931d76472ded14017e59c9e8.html</anchorfile>
      <anchor>aaf363408931d76472ded14017e59c9e8</anchor>
      <arglist>(std::istream &amp;i, basic_json &amp;j)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>parse</name>
      <anchorfile>classnlohmann_1_1basic__json_a15018ade392a844ea32d5188d1a0b9c6.html</anchorfile>
      <anchor>a15018ade392a844ea32d5188d1a0b9c6</anchor>
      <arglist>(InputType &amp;&amp;i, const parser_callback_t cb=nullptr, const bool allow_exceptions=true, const bool ignore_comments=false)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>parse</name>
      <anchorfile>classnlohmann_1_1basic__json_ad832c70af0989389a9a104c21d2d1c5c.html</anchorfile>
      <anchor>ad832c70af0989389a9a104c21d2d1c5c</anchor>
      <arglist>(IteratorType first, IteratorType last, const parser_callback_t cb=nullptr, const bool allow_exceptions=true, const bool ignore_comments=false)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>parse</name>
      <anchorfile>classnlohmann_1_1basic__json_a73cf15644f04fa569f50291049d1bafd.html</anchorfile>
      <anchor>a73cf15644f04fa569f50291049d1bafd</anchor>
      <arglist>(detail::span_input_adapter &amp;&amp;i, const parser_callback_t cb=nullptr, const bool allow_exceptions=true, const bool ignore_comments=false)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>accept</name>
      <anchorfile>classnlohmann_1_1basic__json_a32872afe5bfd040777e3e2bb85f0ca55.html</anchorfile>
      <anchor>a32872afe5bfd040777e3e2bb85f0ca55</anchor>
      <arglist>(InputType &amp;&amp;i, const bool ignore_comments=false)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>accept</name>
      <anchorfile>classnlohmann_1_1basic__json_a47fb596473649332185aedb0a8a6ccc5.html</anchorfile>
      <anchor>a47fb596473649332185aedb0a8a6ccc5</anchor>
      <arglist>(IteratorType first, IteratorType last, const bool ignore_comments=false)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT bool</type>
      <name>accept</name>
      <anchorfile>classnlohmann_1_1basic__json_a6d9e85910b91d02f6807b69b61690a4b.html</anchorfile>
      <anchor>a6d9e85910b91d02f6807b69b61690a4b</anchor>
      <arglist>(detail::span_input_adapter &amp;&amp;i, const bool ignore_comments=false)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>sax_parse</name>
      <anchorfile>classnlohmann_1_1basic__json_a12b382c6407da5543827ce4b24bb5008.html</anchorfile>
      <anchor>a12b382c6407da5543827ce4b24bb5008</anchor>
      <arglist>(InputType &amp;&amp;i, SAX *sax, input_format_t format=input_format_t::json, const bool strict=true, const bool ignore_comments=false)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>sax_parse</name>
      <anchorfile>classnlohmann_1_1basic__json_ab62241c2694a054818edf2f66d72f113.html</anchorfile>
      <anchor>ab62241c2694a054818edf2f66d72f113</anchor>
      <arglist>(IteratorType first, IteratorType last, SAX *sax, input_format_t format=input_format_t::json, const bool strict=true, const bool ignore_comments=false)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>sax_parse</name>
      <anchorfile>classnlohmann_1_1basic__json_aef9ef0a817ecde8bf270653e8706c150.html</anchorfile>
      <anchor>aef9ef0a817ecde8bf270653e8706c150</anchor>
      <arglist>(detail::span_input_adapter &amp;&amp;i, SAX *sax, input_format_t format=input_format_t::json, const bool strict=true, const bool ignore_comments=false)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>get_ptr</name>
      <anchorfile>classnlohmann_1_1basic__json_a85222410e03d8f5c8ff9c78cf9f6b1b6.html</anchorfile>
      <anchor>a85222410e03d8f5c8ff9c78cf9f6b1b6</anchor>
      <arglist>() noexcept -&gt; decltype(std::declval&lt; basic_json_t &amp; &gt;().get_impl_ptr(std::declval&lt; PointerType &gt;()))</arglist>
    </member>
    <member kind="function">
      <type>constexpr auto</type>
      <name>get_ptr</name>
      <anchorfile>classnlohmann_1_1basic__json_afd99a3902a2bb09a306fb854540ae1dd.html</anchorfile>
      <anchor>afd99a3902a2bb09a306fb854540ae1dd</anchor>
      <arglist>() const noexcept -&gt; decltype(std::declval&lt; const basic_json_t &amp; &gt;().get_impl_ptr(std::declval&lt; PointerType &gt;()))</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>get</name>
      <anchorfile>classnlohmann_1_1basic__json_af1650e004db368459b3c0db041b32adc.html</anchorfile>
      <anchor>af1650e004db368459b3c0db041b32adc</anchor>
      <arglist>() const noexcept(noexcept(std::declval&lt; const basic_json_t &amp; &gt;().template get_impl&lt; ValueType &gt;(detail::priority_tag&lt; 4 &gt; {}))) -&gt; decltype(std::declval&lt; const basic_json_t &amp; &gt;().template get_impl&lt; ValueType &gt;(detail::priority_tag&lt; 4 &gt; {}))</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>get</name>
      <anchorfile>classnlohmann_1_1basic__json_a826e180d2457d114ed4c51f4b0737df8.html</anchorfile>
      <anchor>a826e180d2457d114ed4c51f4b0737df8</anchor>
      <arglist>() noexcept -&gt; decltype(std::declval&lt; basic_json_t &amp; &gt;().template get_ptr&lt; PointerType &gt;())</arglist>
    </member>
    <member kind="function">
      <type>ValueType &amp;</type>
      <name>get_to</name>
      <anchorfile>classnlohmann_1_1basic__json_aa1cbe06eb6b15f76e41c7c85081d2638.html</anchorfile>
      <anchor>aa1cbe06eb6b15f76e41c7c85081d2638</anchor>
      <arglist>(ValueType &amp;v) const noexcept(noexcept(JSONSerializer&lt; ValueType &gt;::from_json(std::declval&lt; const basic_json_t &amp; &gt;(), v)))</arglist>
    </member>
    <member kind="function">
      <type>ValueType &amp;</type>
      <name>get_to</name>
      <anchorfile>classnlohmann_1_1basic__json_ac6fd6c12364425e4007ee4d7ecf9fefd.html</anchorfile>
      <anchor>ac6fd6c12364425e4007ee4d7ecf9fefd</anchor>
      <arglist>(ValueType &amp;v) const</arglist>
    </member>
    <member kind="function">
      <type>Array</type>
      <name>get_to</name>
      <anchorfile>classnlohmann_1_1basic__json_a85dd41000e4e7751928fffb44ff4aed4.html</anchorfile>
      <anchor>a85dd41000e4e7751928fffb44ff4aed4</anchor>
      <arglist>(T(&amp;v)[N]) const noexcept(noexcept(JSONSerializer&lt; Array &gt;::from_json(std::declval&lt; const basic_json_t &amp; &gt;(), v)))</arglist>
    </member>
    <member kind="function">
      <type>ReferenceType</type>
      <name>get_ref</name>
      <anchorfile>classnlohmann_1_1basic__json_a14ddc6e0f6d70296bc81f810254076c5.html</anchorfile>
      <anchor>a14ddc6e0f6d70296bc81f810254076c5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>ReferenceType</type>
      <name>get_ref</name>
      <anchorfile>classnlohmann_1_1basic__json_a7f0889072c54f1b899689ed6246238e7.html</anchorfile>
      <anchor>a7f0889072c54f1b899689ed6246238e7</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>JSON_EXPLICIT</type>
      <name>operator ValueType</name>
      <anchorfile>classnlohmann_1_1basic__json_ada1463d8d7ba77865f28f5e83dec7f33.html</anchorfile>
      <anchor>ada1463d8d7ba77865f28f5e83dec7f33</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>binary_t &amp;</type>
      <name>get_binary</name>
      <anchorfile>classnlohmann_1_1basic__json_aab19a246f6bcd27c195bed376cf5e138.html</anchorfile>
      <anchor>aab19a246f6bcd27c195bed376cf5e138</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const binary_t &amp;</type>
      <name>get_binary</name>
      <anchorfile>classnlohmann_1_1basic__json_abbfa5532931abd2d9040cbf4d18a2ca7.html</anchorfile>
      <anchor>abbfa5532931abd2d9040cbf4d18a2ca7</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>binary</name>
      <anchorfile>classnlohmann_1_1basic__json_a3d255dbe024ce2d0fdfb1b4837629091.html</anchorfile>
      <anchor>a3d255dbe024ce2d0fdfb1b4837629091</anchor>
      <arglist>(const typename binary_t::container_type &amp;init)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>binary</name>
      <anchorfile>classnlohmann_1_1basic__json_acd2c506b279049f7d92ad7ae10a2f12b.html</anchorfile>
      <anchor>acd2c506b279049f7d92ad7ae10a2f12b</anchor>
      <arglist>(const typename binary_t::container_type &amp;init, std::uint8_t subtype)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>binary</name>
      <anchorfile>classnlohmann_1_1basic__json_ab085777bbfbfac5a472120b991ef5cf3.html</anchorfile>
      <anchor>ab085777bbfbfac5a472120b991ef5cf3</anchor>
      <arglist>(typename binary_t::container_type &amp;&amp;init)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>binary</name>
      <anchorfile>classnlohmann_1_1basic__json_a012e375f4016b89444c528ed46cce3af.html</anchorfile>
      <anchor>a012e375f4016b89444c528ed46cce3af</anchor>
      <arglist>(typename binary_t::container_type &amp;&amp;init, std::uint8_t subtype)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>array</name>
      <anchorfile>classnlohmann_1_1basic__json_a2c8d8f5741aedadac8f3bffd8f2ce13e.html</anchorfile>
      <anchor>a2c8d8f5741aedadac8f3bffd8f2ce13e</anchor>
      <arglist>(initializer_list_t init={})</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>object</name>
      <anchorfile>classnlohmann_1_1basic__json_a9a4df356e05415438fadf8a15e583903.html</anchorfile>
      <anchor>a9a4df356e05415438fadf8a15e583903</anchor>
      <arglist>(initializer_list_t init={})</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>basic_json</name>
      <anchorfile>classnlohmann_1_1basic__json_a19734fbc9c97d536832892ddacd6b62a.html</anchorfile>
      <anchor>a19734fbc9c97d536832892ddacd6b62a</anchor>
      <arglist>(const value_t v)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>basic_json</name>
      <anchorfile>classnlohmann_1_1basic__json_ace6fbaf6c64f60b61922b5c9d3e61aa6.html</anchorfile>
      <anchor>ace6fbaf6c64f60b61922b5c9d3e61aa6</anchor>
      <arglist>(std::nullptr_t=nullptr) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>basic_json</name>
      <anchorfile>classnlohmann_1_1basic__json_adda5ebaff0503f92c8b3d65cfb610ea5.html</anchorfile>
      <anchor>adda5ebaff0503f92c8b3d65cfb610ea5</anchor>
      <arglist>(CompatibleType &amp;&amp;val) noexcept(noexcept(//NOLINT(bugprone-forwarding-reference-overload, bugprone-exception-escape) JSONSerializer&lt; U &gt;::to_json(std::declval&lt; basic_json_t &amp; &gt;(), std::forward&lt; CompatibleType &gt;(val))))</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>basic_json</name>
      <anchorfile>classnlohmann_1_1basic__json_a40738eb9cb8c5a9ca14ae4b697c29f8e.html</anchorfile>
      <anchor>a40738eb9cb8c5a9ca14ae4b697c29f8e</anchor>
      <arglist>(const BasicJsonType &amp;val)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>basic_json</name>
      <anchorfile>classnlohmann_1_1basic__json_aeac617faf3448d6d2cb33a020be01d37.html</anchorfile>
      <anchor>aeac617faf3448d6d2cb33a020be01d37</anchor>
      <arglist>(initializer_list_t init, bool type_deduction=true, value_t manual_type=value_t::array)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>basic_json</name>
      <anchorfile>classnlohmann_1_1basic__json_afbccea367512a87b5d76e2bd92c5cc85.html</anchorfile>
      <anchor>afbccea367512a87b5d76e2bd92c5cc85</anchor>
      <arglist>(size_type cnt, const basic_json &amp;val)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>basic_json</name>
      <anchorfile>classnlohmann_1_1basic__json_a23e540f473d32f1cf5d3243ea3ad495e.html</anchorfile>
      <anchor>a23e540f473d32f1cf5d3243ea3ad495e</anchor>
      <arglist>(InputIT first, InputIT last)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>basic_json</name>
      <anchorfile>classnlohmann_1_1basic__json_a28524e9c443076ea6dccff8e391354ed.html</anchorfile>
      <anchor>a28524e9c443076ea6dccff8e391354ed</anchor>
      <arglist>(const JsonRef &amp;ref)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>basic_json</name>
      <anchorfile>classnlohmann_1_1basic__json_a603be713183ad63dd8c9e1052c606004.html</anchorfile>
      <anchor>a603be713183ad63dd8c9e1052c606004</anchor>
      <arglist>(const basic_json &amp;other)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>basic_json</name>
      <anchorfile>classnlohmann_1_1basic__json_a642afd9d8886e0dedfc0d5cee2baf57f.html</anchorfile>
      <anchor>a642afd9d8886e0dedfc0d5cee2baf57f</anchor>
      <arglist>(basic_json &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>basic_json &amp;</type>
      <name>operator=</name>
      <anchorfile>classnlohmann_1_1basic__json_a1ae937c299f347a9dcb7f31a8e57762b.html</anchorfile>
      <anchor>a1ae937c299f347a9dcb7f31a8e57762b</anchor>
      <arglist>(basic_json other) noexcept(std::is_nothrow_move_constructible&lt; value_t &gt;::value &amp;&amp;std::is_nothrow_move_assignable&lt; value_t &gt;::value &amp;&amp;std::is_nothrow_move_constructible&lt; json_value &gt;::value &amp;&amp;std::is_nothrow_move_assignable&lt; json_value &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~basic_json</name>
      <anchorfile>classnlohmann_1_1basic__json_a60b643c02a19fa52f99db8215ff58e0f.html</anchorfile>
      <anchor>a60b643c02a19fa52f99db8215ff58e0f</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static iteration_proxy&lt; iterator &gt;</type>
      <name>iterator_wrapper</name>
      <anchorfile>classnlohmann_1_1basic__json_a22e2e5b0e68d9d7c63be2cada5187259.html</anchorfile>
      <anchor>a22e2e5b0e68d9d7c63be2cada5187259</anchor>
      <arglist>(reference ref) noexcept</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static iteration_proxy&lt; const_iterator &gt;</type>
      <name>iterator_wrapper</name>
      <anchorfile>classnlohmann_1_1basic__json_a3710ff8c5a1cbedb4f75b700a8962d5c.html</anchorfile>
      <anchor>a3710ff8c5a1cbedb4f75b700a8962d5c</anchor>
      <arglist>(const_reference ref) noexcept</arglist>
    </member>
    <member kind="function">
      <type>iterator</type>
      <name>begin</name>
      <anchorfile>classnlohmann_1_1basic__json_a23b495b4c282e4afacf382f5b49af7c7.html</anchorfile>
      <anchor>a23b495b4c282e4afacf382f5b49af7c7</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>const_iterator</type>
      <name>begin</name>
      <anchorfile>classnlohmann_1_1basic__json_a4f147be16fcde9f510c4aac89ab511c9.html</anchorfile>
      <anchor>a4f147be16fcde9f510c4aac89ab511c9</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>const_iterator</type>
      <name>cbegin</name>
      <anchorfile>classnlohmann_1_1basic__json_ae508c13e3ad6ce445bcaf24a2bc7d039.html</anchorfile>
      <anchor>ae508c13e3ad6ce445bcaf24a2bc7d039</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>iterator</type>
      <name>end</name>
      <anchorfile>classnlohmann_1_1basic__json_a931267ec3f09eb67e4382f321b2c52bc.html</anchorfile>
      <anchor>a931267ec3f09eb67e4382f321b2c52bc</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>const_iterator</type>
      <name>end</name>
      <anchorfile>classnlohmann_1_1basic__json_a82b5b96f86879a3bac0c713d33178551.html</anchorfile>
      <anchor>a82b5b96f86879a3bac0c713d33178551</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>const_iterator</type>
      <name>cend</name>
      <anchorfile>classnlohmann_1_1basic__json_a3017cf0f1a4673e904e34cfef62e7758.html</anchorfile>
      <anchor>a3017cf0f1a4673e904e34cfef62e7758</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>reverse_iterator</type>
      <name>rbegin</name>
      <anchorfile>classnlohmann_1_1basic__json_aff8e38cd973bc94557fa8d36433c0e4c.html</anchorfile>
      <anchor>aff8e38cd973bc94557fa8d36433c0e4c</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>const_reverse_iterator</type>
      <name>rbegin</name>
      <anchorfile>classnlohmann_1_1basic__json_aab1329f44c8301b7679962726a043549.html</anchorfile>
      <anchor>aab1329f44c8301b7679962726a043549</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>reverse_iterator</type>
      <name>rend</name>
      <anchorfile>classnlohmann_1_1basic__json_a7a328b29b290cc300345376c54f618cb.html</anchorfile>
      <anchor>a7a328b29b290cc300345376c54f618cb</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>const_reverse_iterator</type>
      <name>rend</name>
      <anchorfile>classnlohmann_1_1basic__json_a2e4cbf41d593d41847b90aea55e5e84d.html</anchorfile>
      <anchor>a2e4cbf41d593d41847b90aea55e5e84d</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>const_reverse_iterator</type>
      <name>crbegin</name>
      <anchorfile>classnlohmann_1_1basic__json_a044298d189bdf7e4b36492de9811ddd6.html</anchorfile>
      <anchor>a044298d189bdf7e4b36492de9811ddd6</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>const_reverse_iterator</type>
      <name>crend</name>
      <anchorfile>classnlohmann_1_1basic__json_a223480466a0922267d680ec8f0722d58.html</anchorfile>
      <anchor>a223480466a0922267d680ec8f0722d58</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>iteration_proxy&lt; iterator &gt;</type>
      <name>items</name>
      <anchorfile>classnlohmann_1_1basic__json_a916a6ba75ec7624e9c6c977a52d6fd17.html</anchorfile>
      <anchor>a916a6ba75ec7624e9c6c977a52d6fd17</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>iteration_proxy&lt; const_iterator &gt;</type>
      <name>items</name>
      <anchorfile>classnlohmann_1_1basic__json_a4faaed730a81347f2f01e93f37c73823.html</anchorfile>
      <anchor>a4faaed730a81347f2f01e93f37c73823</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::vector&lt; uint8_t &gt;</type>
      <name>to_cbor</name>
      <anchorfile>classnlohmann_1_1basic__json_adabcf74c9c868da3e04a5546b7705af4.html</anchorfile>
      <anchor>adabcf74c9c868da3e04a5546b7705af4</anchor>
      <arglist>(const basic_json &amp;j)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>to_cbor</name>
      <anchorfile>classnlohmann_1_1basic__json_a2becf604016783e5644eaa7782a08683.html</anchorfile>
      <anchor>a2becf604016783e5644eaa7782a08683</anchor>
      <arglist>(const basic_json &amp;j, detail::output_adapter&lt; uint8_t &gt; o)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>to_cbor</name>
      <anchorfile>classnlohmann_1_1basic__json_a5689672954fd3bc38f2f17e5607064c6.html</anchorfile>
      <anchor>a5689672954fd3bc38f2f17e5607064c6</anchor>
      <arglist>(const basic_json &amp;j, detail::output_adapter&lt; char &gt; o)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::vector&lt; uint8_t &gt;</type>
      <name>to_msgpack</name>
      <anchorfile>classnlohmann_1_1basic__json_a99b15bcaee410426b937eacc6e47d771.html</anchorfile>
      <anchor>a99b15bcaee410426b937eacc6e47d771</anchor>
      <arglist>(const basic_json &amp;j)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>to_msgpack</name>
      <anchorfile>classnlohmann_1_1basic__json_aa8fe46eda86f6f92c5599c20b6c81819.html</anchorfile>
      <anchor>aa8fe46eda86f6f92c5599c20b6c81819</anchor>
      <arglist>(const basic_json &amp;j, detail::output_adapter&lt; uint8_t &gt; o)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>to_msgpack</name>
      <anchorfile>classnlohmann_1_1basic__json_aca5dc0fca52131f3a634372120abfbe7.html</anchorfile>
      <anchor>aca5dc0fca52131f3a634372120abfbe7</anchor>
      <arglist>(const basic_json &amp;j, detail::output_adapter&lt; char &gt; o)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::vector&lt; uint8_t &gt;</type>
      <name>to_ubjson</name>
      <anchorfile>classnlohmann_1_1basic__json_a25355b9719db23b189fb5f6a8f4f16c4.html</anchorfile>
      <anchor>a25355b9719db23b189fb5f6a8f4f16c4</anchor>
      <arglist>(const basic_json &amp;j, const bool use_size=false, const bool use_type=false)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>to_ubjson</name>
      <anchorfile>classnlohmann_1_1basic__json_a19dad92c4fe9e6a289a93195e1230e97.html</anchorfile>
      <anchor>a19dad92c4fe9e6a289a93195e1230e97</anchor>
      <arglist>(const basic_json &amp;j, detail::output_adapter&lt; uint8_t &gt; o, const bool use_size=false, const bool use_type=false)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>to_ubjson</name>
      <anchorfile>classnlohmann_1_1basic__json_a6d133cf7b2e729e9e215edeba5726116.html</anchorfile>
      <anchor>a6d133cf7b2e729e9e215edeba5726116</anchor>
      <arglist>(const basic_json &amp;j, detail::output_adapter&lt; char &gt; o, const bool use_size=false, const bool use_type=false)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::vector&lt; uint8_t &gt;</type>
      <name>to_bson</name>
      <anchorfile>classnlohmann_1_1basic__json_aa62d64781b217372225a0652047d8cf3.html</anchorfile>
      <anchor>aa62d64781b217372225a0652047d8cf3</anchor>
      <arglist>(const basic_json &amp;j)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>to_bson</name>
      <anchorfile>classnlohmann_1_1basic__json_a668e4c2ad9808218a25879700f4aef2b.html</anchorfile>
      <anchor>a668e4c2ad9808218a25879700f4aef2b</anchor>
      <arglist>(const basic_json &amp;j, detail::output_adapter&lt; uint8_t &gt; o)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>to_bson</name>
      <anchorfile>classnlohmann_1_1basic__json_a9ebed178fb7dad1a574bcb7c361fb1b8.html</anchorfile>
      <anchor>a9ebed178fb7dad1a574bcb7c361fb1b8</anchor>
      <arglist>(const basic_json &amp;j, detail::output_adapter&lt; char &gt; o)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_cbor</name>
      <anchorfile>classnlohmann_1_1basic__json_a44dd5635fb2da4710f6cd6e42b72773f.html</anchorfile>
      <anchor>a44dd5635fb2da4710f6cd6e42b72773f</anchor>
      <arglist>(InputType &amp;&amp;i, const bool strict=true, const bool allow_exceptions=true, const cbor_tag_handler_t tag_handler=cbor_tag_handler_t::error)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_cbor</name>
      <anchorfile>classnlohmann_1_1basic__json_aba4f6fc79cc405fb212ea3d992334e71.html</anchorfile>
      <anchor>aba4f6fc79cc405fb212ea3d992334e71</anchor>
      <arglist>(IteratorType first, IteratorType last, const bool strict=true, const bool allow_exceptions=true, const cbor_tag_handler_t tag_handler=cbor_tag_handler_t::error)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_cbor</name>
      <anchorfile>classnlohmann_1_1basic__json_a188755c8cda27e2afb03c016c14125d8.html</anchorfile>
      <anchor>a188755c8cda27e2afb03c016c14125d8</anchor>
      <arglist>(const T *ptr, std::size_t len, const bool strict=true, const bool allow_exceptions=true, const cbor_tag_handler_t tag_handler=cbor_tag_handler_t::error)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_cbor</name>
      <anchorfile>classnlohmann_1_1basic__json_a4a67e47f4bcde55214475f47f8314c1f.html</anchorfile>
      <anchor>a4a67e47f4bcde55214475f47f8314c1f</anchor>
      <arglist>(detail::span_input_adapter &amp;&amp;i, const bool strict=true, const bool allow_exceptions=true, const cbor_tag_handler_t tag_handler=cbor_tag_handler_t::error)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_msgpack</name>
      <anchorfile>classnlohmann_1_1basic__json_adbcab52fca1e25b3311ef14e71a57590.html</anchorfile>
      <anchor>adbcab52fca1e25b3311ef14e71a57590</anchor>
      <arglist>(InputType &amp;&amp;i, const bool strict=true, const bool allow_exceptions=true)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_msgpack</name>
      <anchorfile>classnlohmann_1_1basic__json_a94dbeb08bcd13821512957dcbf3f00bd.html</anchorfile>
      <anchor>a94dbeb08bcd13821512957dcbf3f00bd</anchor>
      <arglist>(IteratorType first, IteratorType last, const bool strict=true, const bool allow_exceptions=true)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_msgpack</name>
      <anchorfile>classnlohmann_1_1basic__json_a0d69d3102639759d5202a6f764760d23.html</anchorfile>
      <anchor>a0d69d3102639759d5202a6f764760d23</anchor>
      <arglist>(const T *ptr, std::size_t len, const bool strict=true, const bool allow_exceptions=true)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_msgpack</name>
      <anchorfile>classnlohmann_1_1basic__json_a84b3d89de8e774a2609dc4c0a1273f62.html</anchorfile>
      <anchor>a84b3d89de8e774a2609dc4c0a1273f62</anchor>
      <arglist>(detail::span_input_adapter &amp;&amp;i, const bool strict=true, const bool allow_exceptions=true)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_ubjson</name>
      <anchorfile>classnlohmann_1_1basic__json_aa81f62db69978b90ff701f05c72e03a7.html</anchorfile>
      <anchor>aa81f62db69978b90ff701f05c72e03a7</anchor>
      <arglist>(InputType &amp;&amp;i, const bool strict=true, const bool allow_exceptions=true)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_ubjson</name>
      <anchorfile>classnlohmann_1_1basic__json_a9dbb422350fed520ce8c1ca8762c0251.html</anchorfile>
      <anchor>a9dbb422350fed520ce8c1ca8762c0251</anchor>
      <arglist>(IteratorType first, IteratorType last, const bool strict=true, const bool allow_exceptions=true)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_ubjson</name>
      <anchorfile>classnlohmann_1_1basic__json_ab6eeda23c34cee79d8d72f1a8691a8de.html</anchorfile>
      <anchor>ab6eeda23c34cee79d8d72f1a8691a8de</anchor>
      <arglist>(const T *ptr, std::size_t len, const bool strict=true, const bool allow_exceptions=true)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_ubjson</name>
      <anchorfile>classnlohmann_1_1basic__json_a47201396899371881a46562bffb922da.html</anchorfile>
      <anchor>a47201396899371881a46562bffb922da</anchor>
      <arglist>(detail::span_input_adapter &amp;&amp;i, const bool strict=true, const bool allow_exceptions=true)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_bson</name>
      <anchorfile>classnlohmann_1_1basic__json_a4e02793f2691aa29ab7cb69fddafbf5c.html</anchorfile>
      <anchor>a4e02793f2691aa29ab7cb69fddafbf5c</anchor>
      <arglist>(InputType &amp;&amp;i, const bool strict=true, const bool allow_exceptions=true)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_bson</name>
      <anchorfile>classnlohmann_1_1basic__json_a4118d0ec23d9eeafc236b9524d220e94.html</anchorfile>
      <anchor>a4118d0ec23d9eeafc236b9524d220e94</anchor>
      <arglist>(IteratorType first, IteratorType last, const bool strict=true, const bool allow_exceptions=true)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_bson</name>
      <anchorfile>classnlohmann_1_1basic__json_ae2612581e4788ddffc2f45d5e4cc04fc.html</anchorfile>
      <anchor>ae2612581e4788ddffc2f45d5e4cc04fc</anchor>
      <arglist>(const T *ptr, std::size_t len, const bool strict=true, const bool allow_exceptions=true)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>from_bson</name>
      <anchorfile>classnlohmann_1_1basic__json_a2afe89884edf72412a0624982324755d.html</anchorfile>
      <anchor>a2afe89884edf72412a0624982324755d</anchor>
      <arglist>(detail::span_input_adapter &amp;&amp;i, const bool strict=true, const bool allow_exceptions=true)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static JSON_HEDLEY_WARN_UNUSED_RESULT basic_json</type>
      <name>diff</name>
      <anchorfile>classnlohmann_1_1basic__json_a1c1f21327df91a4dd6c5f5a107240385.html</anchorfile>
      <anchor>a1c1f21327df91a4dd6c5f5a107240385</anchor>
      <arglist>(const basic_json &amp;source, const basic_json &amp;target, const std::string &amp;path=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type>basic_json</type>
      <name>patch</name>
      <anchorfile>classnlohmann_1_1basic__json_adcc786998f220a5b3083ee8a37c4553e.html</anchorfile>
      <anchor>adcc786998f220a5b3083ee8a37c4553e</anchor>
      <arglist>(const basic_json &amp;json_patch) const</arglist>
    </member>
    <member kind="function">
      <type>string_t</type>
      <name>dump</name>
      <anchorfile>classnlohmann_1_1basic__json_a476756fb08e7f2416aad116d137977f4.html</anchorfile>
      <anchor>a476756fb08e7f2416aad116d137977f4</anchor>
      <arglist>(const int indent=-1, const char indent_char=&apos; &apos;, const bool ensure_ascii=false, const error_handler_t error_handler=error_handler_t::strict) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr value_t</type>
      <name>type</name>
      <anchorfile>classnlohmann_1_1basic__json_a5b7c4b35a0ad9f97474912a08965d7ad.html</anchorfile>
      <anchor>a5b7c4b35a0ad9f97474912a08965d7ad</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_primitive</name>
      <anchorfile>classnlohmann_1_1basic__json_a548d2d4013da24e7d7510d90febc80c4.html</anchorfile>
      <anchor>a548d2d4013da24e7d7510d90febc80c4</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_structured</name>
      <anchorfile>classnlohmann_1_1basic__json_a4e05a7d5deec758f1d830741b68b4249.html</anchorfile>
      <anchor>a4e05a7d5deec758f1d830741b68b4249</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_null</name>
      <anchorfile>classnlohmann_1_1basic__json_aedc7afad96292b5ab61a2e0ad3067f5f.html</anchorfile>
      <anchor>aedc7afad96292b5ab61a2e0ad3067f5f</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_boolean</name>
      <anchorfile>classnlohmann_1_1basic__json_a911b11e855e685fa59ea1d111490b36b.html</anchorfile>
      <anchor>a911b11e855e685fa59ea1d111490b36b</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_number</name>
      <anchorfile>classnlohmann_1_1basic__json_abd47ac8eba63833152795280f75b5851.html</anchorfile>
      <anchor>abd47ac8eba63833152795280f75b5851</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_number_integer</name>
      <anchorfile>classnlohmann_1_1basic__json_ac4b4acf2c0ad075c0dc125a65c102362.html</anchorfile>
      <anchor>ac4b4acf2c0ad075c0dc125a65c102362</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_number_unsigned</name>
      <anchorfile>classnlohmann_1_1basic__json_a5493f2ed1e07b0ece428bd5a47e2fb95.html</anchorfile>
      <anchor>a5493f2ed1e07b0ece428bd5a47e2fb95</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_number_float</name>
      <anchorfile>classnlohmann_1_1basic__json_a116cdb9300b56519fc9cf756609296cb.html</anchorfile>
      <anchor>a116cdb9300b56519fc9cf756609296cb</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_object</name>
      <anchorfile>classnlohmann_1_1basic__json_a57e8411a770a6263d6d8f2116c37f3aa.html</anchorfile>
      <anchor>a57e8411a770a6263d6d8f2116c37f3aa</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_array</name>
      <anchorfile>classnlohmann_1_1basic__json_ab5b70d60a636b9c5e10f6c8caac60b9e.html</anchorfile>
      <anchor>ab5b70d60a636b9c5e10f6c8caac60b9e</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_string</name>
      <anchorfile>classnlohmann_1_1basic__json_ab303d17366c26fca12242c7f8def1bb7.html</anchorfile>
      <anchor>ab303d17366c26fca12242c7f8def1bb7</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_binary</name>
      <anchorfile>classnlohmann_1_1basic__json_a9576224f7b3ff812a308e7c1e784ea80.html</anchorfile>
      <anchor>a9576224f7b3ff812a308e7c1e784ea80</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>is_discarded</name>
      <anchorfile>classnlohmann_1_1basic__json_aecaaa0613d3f3a5b49b34b02afc5f85d.html</anchorfile>
      <anchor>aecaaa0613d3f3a5b49b34b02afc5f85d</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator value_t</name>
      <anchorfile>classnlohmann_1_1basic__json_a6d4b8df10ecc533a50823e8805f4a873.html</anchorfile>
      <anchor>a6d4b8df10ecc533a50823e8805f4a873</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>at</name>
      <anchorfile>classnlohmann_1_1basic__json_a52b18a5b7e68652c65b070900c438c6e.html</anchorfile>
      <anchor>a52b18a5b7e68652c65b070900c438c6e</anchor>
      <arglist>(size_type idx)</arglist>
    </member>
    <member kind="function">
      <type>const_reference</type>
      <name>at</name>
      <anchorfile>classnlohmann_1_1basic__json_aeb18fe2b8a5dbff4ccf2848de854c3ac.html</anchorfile>
      <anchor>aeb18fe2b8a5dbff4ccf2848de854c3ac</anchor>
      <arglist>(size_type idx) const</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>at</name>
      <anchorfile>classnlohmann_1_1basic__json_a239e942da82f2597d0cf5ec806f5bc0d.html</anchorfile>
      <anchor>a239e942da82f2597d0cf5ec806f5bc0d</anchor>
      <arglist>(const typename object_t::key_type &amp;key)</arglist>
    </member>
    <member kind="function">
      <type>const_reference</type>
      <name>at</name>
      <anchorfile>classnlohmann_1_1basic__json_a229964ee10c92ba89ae4fba786fe6b50.html</anchorfile>
      <anchor>a229964ee10c92ba89ae4fba786fe6b50</anchor>
      <arglist>(const typename object_t::key_type &amp;key) const</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>operator[]</name>
      <anchorfile>classnlohmann_1_1basic__json_a9ea67fc1ef0ccc42e1d5388fe0416ae5.html</anchorfile>
      <anchor>a9ea67fc1ef0ccc42e1d5388fe0416ae5</anchor>
      <arglist>(size_type idx)</arglist>
    </member>
    <member kind="function">
      <type>const_reference</type>
      <name>operator[]</name>
      <anchorfile>classnlohmann_1_1basic__json_ad21d96f490fa1aa8605fba8dadcce319.html</anchorfile>
      <anchor>ad21d96f490fa1aa8605fba8dadcce319</anchor>
      <arglist>(size_type idx) const</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>operator[]</name>
      <anchorfile>classnlohmann_1_1basic__json_a3f45f3820c456ad2e3f3df2926564151.html</anchorfile>
      <anchor>a3f45f3820c456ad2e3f3df2926564151</anchor>
      <arglist>(const typename object_t::key_type &amp;key)</arglist>
    </member>
    <member kind="function">
      <type>const_reference</type>
      <name>operator[]</name>
      <anchorfile>classnlohmann_1_1basic__json_acb5b489310f4e0ce2d5fb29b73fb59d3.html</anchorfile>
      <anchor>acb5b489310f4e0ce2d5fb29b73fb59d3</anchor>
      <arglist>(const typename object_t::key_type &amp;key) const</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>operator[]</name>
      <anchorfile>classnlohmann_1_1basic__json_abc94831476f7b4d3efe6f2e9036c7188.html</anchorfile>
      <anchor>abc94831476f7b4d3efe6f2e9036c7188</anchor>
      <arglist>(T *key)</arglist>
    </member>
    <member kind="function">
      <type>const_reference</type>
      <name>operator[]</name>
      <anchorfile>classnlohmann_1_1basic__json_a11bbe874496eb7b29a5549e0637de59e.html</anchorfile>
      <anchor>a11bbe874496eb7b29a5549e0637de59e</anchor>
      <arglist>(T *key) const</arglist>
    </member>
    <member kind="function">
      <type>ValueType</type>
      <name>value</name>
      <anchorfile>classnlohmann_1_1basic__json_a11641b35219676b225d9bd15c7677659.html</anchorfile>
      <anchor>a11641b35219676b225d9bd15c7677659</anchor>
      <arglist>(const typename object_t::key_type &amp;key, const ValueType &amp;default_value) const</arglist>
    </member>
    <member kind="function">
      <type>string_t</type>
      <name>value</name>
      <anchorfile>classnlohmann_1_1basic__json_adcfdefe95d5c2471a5c97e911d46ee88.html</anchorfile>
      <anchor>adcfdefe95d5c2471a5c97e911d46ee88</anchor>
      <arglist>(const typename object_t::key_type &amp;key, const char *default_value) const</arglist>
    </member>
    <member kind="function">
      <type>ValueType</type>
      <name>value</name>
      <anchorfile>classnlohmann_1_1basic__json_a36bd6765ccd8aeeeb4e49a766ba639df.html</anchorfile>
      <anchor>a36bd6765ccd8aeeeb4e49a766ba639df</anchor>
      <arglist>(const json_pointer &amp;ptr, const ValueType &amp;default_value) const</arglist>
    </member>
    <member kind="function">
      <type>string_t</type>
      <name>value</name>
      <anchorfile>classnlohmann_1_1basic__json_a4658a584571c4180656f5268969ed0a1.html</anchorfile>
      <anchor>a4658a584571c4180656f5268969ed0a1</anchor>
      <arglist>(const json_pointer &amp;ptr, const char *default_value) const</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>front</name>
      <anchorfile>classnlohmann_1_1basic__json_a5417ca43ae5e7a3a2f82eee2d915c6ed.html</anchorfile>
      <anchor>a5417ca43ae5e7a3a2f82eee2d915c6ed</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const_reference</type>
      <name>front</name>
      <anchorfile>classnlohmann_1_1basic__json_a5ac8f974c178cf9326b6765e22f50eb6.html</anchorfile>
      <anchor>a5ac8f974c178cf9326b6765e22f50eb6</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>back</name>
      <anchorfile>classnlohmann_1_1basic__json_a30914ad0767ccdc3633f88a906ed7dfa.html</anchorfile>
      <anchor>a30914ad0767ccdc3633f88a906ed7dfa</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const_reference</type>
      <name>back</name>
      <anchorfile>classnlohmann_1_1basic__json_aac965b84ea43ccd8aef9caefef02794a.html</anchorfile>
      <anchor>aac965b84ea43ccd8aef9caefef02794a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>IteratorType</type>
      <name>erase</name>
      <anchorfile>classnlohmann_1_1basic__json_a494632b69bbe1d0153d3bedad0901b8e.html</anchorfile>
      <anchor>a494632b69bbe1d0153d3bedad0901b8e</anchor>
      <arglist>(IteratorType pos)</arglist>
    </member>
    <member kind="function">
      <type>IteratorType</type>
      <name>erase</name>
      <anchorfile>classnlohmann_1_1basic__json_a8ac83750e267e37d5d47591eb44cce42.html</anchorfile>
      <anchor>a8ac83750e267e37d5d47591eb44cce42</anchor>
      <arglist>(IteratorType first, IteratorType last)</arglist>
    </member>
    <member kind="function">
      <type>size_type</type>
      <name>erase</name>
      <anchorfile>classnlohmann_1_1basic__json_af72b1c9d1502b02a49a0cb9db9f980ea.html</anchorfile>
      <anchor>af72b1c9d1502b02a49a0cb9db9f980ea</anchor>
      <arglist>(const typename object_t::key_type &amp;key)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>erase</name>
      <anchorfile>classnlohmann_1_1basic__json_a221b943d3228488c14225e55f726cc26.html</anchorfile>
      <anchor>a221b943d3228488c14225e55f726cc26</anchor>
      <arglist>(const size_type idx)</arglist>
    </member>
    <member kind="function">
      <type>iterator</type>
      <name>find</name>
      <anchorfile>classnlohmann_1_1basic__json_acdf9b3aab82dcf443dd91ca5ec06b80c.html</anchorfile>
      <anchor>acdf9b3aab82dcf443dd91ca5ec06b80c</anchor>
      <arglist>(KeyT &amp;&amp;key)</arglist>
    </member>
    <member kind="function">
      <type>const_iterator</type>
      <name>find</name>
      <anchorfile>classnlohmann_1_1basic__json_a17a516671c29a69bb2e11ca12030316b.html</anchorfile>
      <anchor>a17a516671c29a69bb2e11ca12030316b</anchor>
      <arglist>(KeyT &amp;&amp;key) const</arglist>
    </member>
    <member kind="function">
      <type>size_type</type>
      <name>count</name>
      <anchorfile>classnlohmann_1_1basic__json_aba5ec6d1e37eda6b11eba491a1e5237e.html</anchorfile>
      <anchor>aba5ec6d1e37eda6b11eba491a1e5237e</anchor>
      <arglist>(KeyT &amp;&amp;key) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>contains</name>
      <anchorfile>classnlohmann_1_1basic__json_a02c9bc4d0f33b7dec20b2798301d6971.html</anchorfile>
      <anchor>a02c9bc4d0f33b7dec20b2798301d6971</anchor>
      <arglist>(KeyT &amp;&amp;key) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>contains</name>
      <anchorfile>classnlohmann_1_1basic__json_adb82c1f34c73486e013da71ae369e597.html</anchorfile>
      <anchor>adb82c1f34c73486e013da71ae369e597</anchor>
      <arglist>(const json_pointer &amp;ptr) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>empty</name>
      <anchorfile>classnlohmann_1_1basic__json_a5c99855f3e35ff35558cb46139b785f8.html</anchorfile>
      <anchor>a5c99855f3e35ff35558cb46139b785f8</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>size_type</type>
      <name>size</name>
      <anchorfile>classnlohmann_1_1basic__json_a33c7c8638bb0b12e6d1b69d8106dd2e0.html</anchorfile>
      <anchor>a33c7c8638bb0b12e6d1b69d8106dd2e0</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>size_type</type>
      <name>max_size</name>
      <anchorfile>classnlohmann_1_1basic__json_a1b46c6631e30b8394e89bd1546d69736.html</anchorfile>
      <anchor>a1b46c6631e30b8394e89bd1546d69736</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>operator[]</name>
      <anchorfile>classnlohmann_1_1basic__json_a0d3deaa73b3644b4da3f8ef3172cd8d2.html</anchorfile>
      <anchor>a0d3deaa73b3644b4da3f8ef3172cd8d2</anchor>
      <arglist>(const json_pointer &amp;ptr)</arglist>
    </member>
    <member kind="function">
      <type>const_reference</type>
      <name>operator[]</name>
      <anchorfile>classnlohmann_1_1basic__json_a0a691c29eb7c4acd91ccb498a79cd3ee.html</anchorfile>
      <anchor>a0a691c29eb7c4acd91ccb498a79cd3ee</anchor>
      <arglist>(const json_pointer &amp;ptr) const</arglist>
    </member>
    <member kind="function">
      <type>reference</type>
      <name>at</name>
      <anchorfile>classnlohmann_1_1basic__json_aa014a978f8b6c085db8825faa8dad320.html</anchorfile>
      <anchor>aa014a978f8b6c085db8825faa8dad320</anchor>
      <arglist>(const json_pointer &amp;ptr)</arglist>
    </member>
    <member kind="function">
      <type>const_reference</type>
      <name>at</name>
      <anchorfile>classnlohmann_1_1basic__json_a8284b9c1d4d0830151eaa000f907b2e6.html</anchorfile>
      <anchor>a8284b9c1d4d0830151eaa000f907b2e6</anchor>
      <arglist>(const json_pointer &amp;ptr) const</arglist>
    </member>
    <member kind="function">
      <type>basic_json</type>
      <name>flatten</name>
      <anchorfile>classnlohmann_1_1basic__json_ab7aa6e048e659481a036f2d872c7cba6.html</anchorfile>
      <anchor>ab7aa6e048e659481a036f2d872c7cba6</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>basic_json</type>
      <name>unflatten</name>
      <anchorfile>classnlohmann_1_1basic__json_adea158bff8642202420898f6322da479.html</anchorfile>
      <anchor>adea158bff8642202420898f6322da479</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>merge_patch</name>
      <anchorfile>classnlohmann_1_1basic__json_a844a77cb154752d12118f10af26d54cb.html</anchorfile>
      <anchor>a844a77cb154752d12118f10af26d54cb</anchor>
      <arglist>(const basic_json &amp;apply_patch)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>nlohmann::byte_container_with_subtype</name>
    <filename>classnlohmann_1_1byte__container__with__subtype.html</filename>
    <templarg></templarg>
    <member kind="typedef">
      <type>BinaryType</type>
      <name>container_type</name>
      <anchorfile>classnlohmann_1_1byte__container__with__subtype_a4d27e8633c5a5e3b49dd4ccb06515713.html</anchorfile>
      <anchor>a4d27e8633c5a5e3b49dd4ccb06515713</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>byte_container_with_subtype</name>
      <anchorfile>classnlohmann_1_1byte__container__with__subtype_a89c78caf8c7b54dc1bcfa4b0b23d2fc8.html</anchorfile>
      <anchor>a89c78caf8c7b54dc1bcfa4b0b23d2fc8</anchor>
      <arglist>() noexcept(noexcept(container_type()))</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>byte_container_with_subtype</name>
      <anchorfile>classnlohmann_1_1byte__container__with__subtype_a640b6dd55847e86dbb936f97b946170e.html</anchorfile>
      <anchor>a640b6dd55847e86dbb936f97b946170e</anchor>
      <arglist>(const container_type &amp;b) noexcept(noexcept(container_type(b)))</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>byte_container_with_subtype</name>
      <anchorfile>classnlohmann_1_1byte__container__with__subtype_a9bd3e08ec6d3ed9014ad7d83eca5e3b3.html</anchorfile>
      <anchor>a9bd3e08ec6d3ed9014ad7d83eca5e3b3</anchor>
      <arglist>(const container_type &amp;b, std::uint8_t subtype_) noexcept(noexcept(container_type(b)))</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>byte_container_with_subtype</name>
      <anchorfile>classnlohmann_1_1byte__container__with__subtype_a73dcae1798eab1b496936bfae7b4b9c0.html</anchorfile>
      <anchor>a73dcae1798eab1b496936bfae7b4b9c0</anchor>
      <arglist>(container_type &amp;&amp;b) noexcept(noexcept(container_type(std::move(b))))</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>byte_container_with_subtype</name>
      <anchorfile>classnlohmann_1_1byte__container__with__subtype_aa9e66c61f46afdd9a9c3617967c4c797.html</anchorfile>
      <anchor>aa9e66c61f46afdd9a9c3617967c4c797</anchor>
      <arglist>(container_type &amp;&amp;b, std::uint8_t subtype_) noexcept(noexcept(container_type(std::move(b))))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>clear_subtype</name>
      <anchorfile>classnlohmann_1_1byte__container__with__subtype_a7b122b28ff2b8680557ca44ac9748e49.html</anchorfile>
      <anchor>a7b122b28ff2b8680557ca44ac9748e49</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>has_subtype</name>
      <anchorfile>classnlohmann_1_1byte__container__with__subtype_a9fc42fb07003bf7048c2f1fc79478e02.html</anchorfile>
      <anchor>a9fc42fb07003bf7048c2f1fc79478e02</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classnlohmann_1_1byte__container__with__subtype_a760bf39cc5477bc663d8bb3c44aabf6a.html</anchorfile>
      <anchor>a760bf39cc5477bc663d8bb3c44aabf6a</anchor>
      <arglist>(const byte_container_with_subtype &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classnlohmann_1_1byte__container__with__subtype_aee67fde9d3d571a07d5bb35df21c0555.html</anchorfile>
      <anchor>aee67fde9d3d571a07d5bb35df21c0555</anchor>
      <arglist>(const byte_container_with_subtype &amp;rhs) const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>set_subtype</name>
      <anchorfile>classnlohmann_1_1byte__container__with__subtype_a63227e70c8b976fd6f65bb2d2d7dd021.html</anchorfile>
      <anchor>a63227e70c8b976fd6f65bb2d2d7dd021</anchor>
      <arglist>(std::uint8_t subtype_) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::uint8_t</type>
      <name>subtype</name>
      <anchorfile>classnlohmann_1_1byte__container__with__subtype_ac3ca9d09e55342f9588404e1dc2222f0.html</anchorfile>
      <anchor>ac3ca9d09e55342f9588404e1dc2222f0</anchor>
      <arglist>() const noexcept</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>std::hash&lt; nlohmann::json &gt;</name>
    <filename>structstd_1_1hash_3_01nlohmann_1_1json_01_4.html</filename>
    <member kind="function">
      <type>std::size_t</type>
      <name>operator()</name>
      <anchorfile>structstd_1_1hash_3_01nlohmann_1_1json_01_4_aec1567d1fa47dbe5b77954dce3a55b64.html</anchorfile>
      <anchor>aec1567d1fa47dbe5b77954dce3a55b64</anchor>
      <arglist>(const nlohmann::json &amp;j) const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>nlohmann::json_pointer</name>
    <filename>classnlohmann_1_1json__pointer.html</filename>
    <templarg></templarg>
    <member kind="function">
      <type></type>
      <name>json_pointer</name>
      <anchorfile>classnlohmann_1_1json__pointer_a7f32d7c62841f0c4a6784cf741a6e4f8.html</anchorfile>
      <anchor>a7f32d7c62841f0c4a6784cf741a6e4f8</anchor>
      <arglist>(const std::string &amp;s=&quot;&quot;)</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>back</name>
      <anchorfile>classnlohmann_1_1json__pointer_a213bc67c32a30c68ac6bf06f5195d482.html</anchorfile>
      <anchor>a213bc67c32a30c68ac6bf06f5195d482</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>empty</name>
      <anchorfile>classnlohmann_1_1json__pointer_a649252bda4a2e75a0915b11a25d8bcc3.html</anchorfile>
      <anchor>a649252bda4a2e75a0915b11a25d8bcc3</anchor>
      <arglist>() const noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator std::string</name>
      <anchorfile>classnlohmann_1_1json__pointer_ae9015c658f99cf3d48a8563accc79988.html</anchorfile>
      <anchor>ae9015c658f99cf3d48a8563accc79988</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>json_pointer &amp;</type>
      <name>operator/=</name>
      <anchorfile>classnlohmann_1_1json__pointer_a7395bd0af29ac23fd3f21543c935cdfa.html</anchorfile>
      <anchor>a7395bd0af29ac23fd3f21543c935cdfa</anchor>
      <arglist>(const json_pointer &amp;ptr)</arglist>
    </member>
    <member kind="function">
      <type>json_pointer &amp;</type>
      <name>operator/=</name>
      <anchorfile>classnlohmann_1_1json__pointer_a7de51480324eb1c5a89ed552cd699875.html</anchorfile>
      <anchor>a7de51480324eb1c5a89ed552cd699875</anchor>
      <arglist>(std::size_t array_idx)</arglist>
    </member>
    <member kind="function">
      <type>json_pointer &amp;</type>
      <name>operator/=</name>
      <anchorfile>classnlohmann_1_1json__pointer_abdd21567b2b1d69329af0f520335e68b.html</anchorfile>
      <anchor>abdd21567b2b1d69329af0f520335e68b</anchor>
      <arglist>(std::string token)</arglist>
    </member>
    <member kind="function">
      <type>json_pointer</type>
      <name>parent_pointer</name>
      <anchorfile>classnlohmann_1_1json__pointer_afdaacce1edb7145e0434e014f0e8685a.html</anchorfile>
      <anchor>afdaacce1edb7145e0434e014f0e8685a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>pop_back</name>
      <anchorfile>classnlohmann_1_1json__pointer_a4b1ee4d511ca195bed896a3da47e264c.html</anchorfile>
      <anchor>a4b1ee4d511ca195bed896a3da47e264c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>push_back</name>
      <anchorfile>classnlohmann_1_1json__pointer_a697d12b5bd6205f8866691b166b7c7dc.html</anchorfile>
      <anchor>a697d12b5bd6205f8866691b166b7c7dc</anchor>
      <arglist>(const std::string &amp;token)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>push_back</name>
      <anchorfile>classnlohmann_1_1json__pointer_ac228b13596d3c34185da9fe61b570194.html</anchorfile>
      <anchor>ac228b13596d3c34185da9fe61b570194</anchor>
      <arglist>(std::string &amp;&amp;token)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>to_string</name>
      <anchorfile>classnlohmann_1_1json__pointer_a3d4b15d32d096e3776c5d2c773b524f5.html</anchorfile>
      <anchor>a3d4b15d32d096e3776c5d2c773b524f5</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="friend" protection="private">
      <type>friend class</type>
      <name>basic_json</name>
      <anchorfile>classnlohmann_1_1json__pointer_ada3100cdb8700566051828f1355fa745.html</anchorfile>
      <anchor>ada3100cdb8700566051828f1355fa745</anchor>
      <arglist></arglist>
    </member>
    <member kind="friend" protection="private">
      <type>friend bool</type>
      <name>operator!=</name>
      <anchorfile>classnlohmann_1_1json__pointer_a6779edcf28e6f018a3bbb29c0b4b5e1e.html</anchorfile>
      <anchor>a6779edcf28e6f018a3bbb29c0b4b5e1e</anchor>
      <arglist>(json_pointer const &amp;lhs, json_pointer const &amp;rhs) noexcept</arglist>
    </member>
    <member kind="friend">
      <type>friend json_pointer</type>
      <name>operator/</name>
      <anchorfile>classnlohmann_1_1json__pointer_a90a11fe6c7f37b1746a3ff9cb24b0d53.html</anchorfile>
      <anchor>a90a11fe6c7f37b1746a3ff9cb24b0d53</anchor>
      <arglist>(const json_pointer &amp;lhs, const json_pointer &amp;rhs)</arglist>
    </member>
    <member kind="friend">
      <type>friend json_pointer</type>
      <name>operator/</name>
      <anchorfile>classnlohmann_1_1json__pointer_af5a4bc4f82113c271c9a0cd4d3b5f31c.html</anchorfile>
      <anchor>af5a4bc4f82113c271c9a0cd4d3b5f31c</anchor>
      <arglist>(const json_pointer &amp;ptr, std::size_t array_idx)</arglist>
    </member>
    <member kind="friend">
      <type>friend json_pointer</type>
      <name>operator/</name>
      <anchorfile>classnlohmann_1_1json__pointer_a926c9065dbed1bedc17857a813f7a46f.html</anchorfile>
      <anchor>a926c9065dbed1bedc17857a813f7a46f</anchor>
      <arglist>(const json_pointer &amp;ptr, std::string token)</arglist>
    </member>
    <member kind="friend" protection="private">
      <type>friend bool</type>
      <name>operator==</name>
      <anchorfile>classnlohmann_1_1json__pointer_a4667ef558c8c3f8a646bfda0c6654653.html</anchorfile>
      <anchor>a4667ef558c8c3f8a646bfda0c6654653</anchor>
      <arglist>(json_pointer const &amp;lhs, json_pointer const &amp;rhs) noexcept</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>nlohmann::json_sax</name>
    <filename>structnlohmann_1_1json__sax.html</filename>
    <templarg></templarg>
    <member kind="typedef">
      <type>typename BasicJsonType::binary_t</type>
      <name>binary_t</name>
      <anchorfile>structnlohmann_1_1json__sax_a0ef406ba81eef08aadf4a9ef48d742bd.html</anchorfile>
      <anchor>a0ef406ba81eef08aadf4a9ef48d742bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename BasicJsonType::number_float_t</type>
      <name>number_float_t</name>
      <anchorfile>structnlohmann_1_1json__sax_a390c209bffd8c4800c8f3076dc465a20.html</anchorfile>
      <anchor>a390c209bffd8c4800c8f3076dc465a20</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename BasicJsonType::number_integer_t</type>
      <name>number_integer_t</name>
      <anchorfile>structnlohmann_1_1json__sax_a0cef30121f02b7fee85e9708148ea0aa.html</anchorfile>
      <anchor>a0cef30121f02b7fee85e9708148ea0aa</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename BasicJsonType::number_unsigned_t</type>
      <name>number_unsigned_t</name>
      <anchorfile>structnlohmann_1_1json__sax_a32028cc056ae0f43aaae331cdbbbf856.html</anchorfile>
      <anchor>a32028cc056ae0f43aaae331cdbbbf856</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename BasicJsonType::string_t</type>
      <name>string_t</name>
      <anchorfile>structnlohmann_1_1json__sax_ae01977a9f3c5b3667b7a2929ed91061e.html</anchorfile>
      <anchor>ae01977a9f3c5b3667b7a2929ed91061e</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>json_sax</name>
      <anchorfile>structnlohmann_1_1json__sax_aac0f445c1173fa731104d93496a7e780.html</anchorfile>
      <anchor>aac0f445c1173fa731104d93496a7e780</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>json_sax</name>
      <anchorfile>structnlohmann_1_1json__sax_a8358c063b2d7222b3aafa62fded04403.html</anchorfile>
      <anchor>a8358c063b2d7222b3aafa62fded04403</anchor>
      <arglist>(const json_sax &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>json_sax</name>
      <anchorfile>structnlohmann_1_1json__sax_a62dae3713ca4914265904e30cd6b6347.html</anchorfile>
      <anchor>a62dae3713ca4914265904e30cd6b6347</anchor>
      <arglist>(json_sax &amp;&amp;) noexcept=default</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~json_sax</name>
      <anchorfile>structnlohmann_1_1json__sax_af31bacfa81aa7818d8639d1da65c8eb5.html</anchorfile>
      <anchor>af31bacfa81aa7818d8639d1da65c8eb5</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>binary</name>
      <anchorfile>structnlohmann_1_1json__sax_a38c2dbde07138cc436ea7fbf22c1e92d.html</anchorfile>
      <anchor>a38c2dbde07138cc436ea7fbf22c1e92d</anchor>
      <arglist>(binary_t &amp;val)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>boolean</name>
      <anchorfile>structnlohmann_1_1json__sax_a82ed080814fa656191a537284bb0c575.html</anchorfile>
      <anchor>a82ed080814fa656191a537284bb0c575</anchor>
      <arglist>(bool val)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>end_array</name>
      <anchorfile>structnlohmann_1_1json__sax_a235ee975617f28e6a996d1e36a282312.html</anchorfile>
      <anchor>a235ee975617f28e6a996d1e36a282312</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>end_object</name>
      <anchorfile>structnlohmann_1_1json__sax_ad0c722d53ff97be700ccf6a9468bd456.html</anchorfile>
      <anchor>ad0c722d53ff97be700ccf6a9468bd456</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>key</name>
      <anchorfile>structnlohmann_1_1json__sax_a2e0c7ecd80b18d18a8cc76f71cfc2028.html</anchorfile>
      <anchor>a2e0c7ecd80b18d18a8cc76f71cfc2028</anchor>
      <arglist>(string_t &amp;val)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>null</name>
      <anchorfile>structnlohmann_1_1json__sax_a0ad26edef3f8d530dcec3192bba82df6.html</anchorfile>
      <anchor>a0ad26edef3f8d530dcec3192bba82df6</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>number_float</name>
      <anchorfile>structnlohmann_1_1json__sax_ae7c31614e8a82164d2d7f8dbf4671b25.html</anchorfile>
      <anchor>ae7c31614e8a82164d2d7f8dbf4671b25</anchor>
      <arglist>(number_float_t val, const string_t &amp;s)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>number_integer</name>
      <anchorfile>structnlohmann_1_1json__sax_affa7a78b8e9cc9cc3ac99927143142a5.html</anchorfile>
      <anchor>affa7a78b8e9cc9cc3ac99927143142a5</anchor>
      <arglist>(number_integer_t val)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>number_unsigned</name>
      <anchorfile>structnlohmann_1_1json__sax_ad9b253083e0509923ba195136f49face.html</anchorfile>
      <anchor>ad9b253083e0509923ba195136f49face</anchor>
      <arglist>(number_unsigned_t val)=0</arglist>
    </member>
    <member kind="function">
      <type>json_sax &amp;</type>
      <name>operator=</name>
      <anchorfile>structnlohmann_1_1json__sax_a1a90dae111cf189ac9ad340a60b199b6.html</anchorfile>
      <anchor>a1a90dae111cf189ac9ad340a60b199b6</anchor>
      <arglist>(const json_sax &amp;)=default</arglist>
    </member>
    <member kind="function">
      <type>json_sax &amp;</type>
      <name>operator=</name>
      <anchorfile>structnlohmann_1_1json__sax_ac74584e3dc41f2465e69b37d7d2bf694.html</anchorfile>
      <anchor>ac74584e3dc41f2465e69b37d7d2bf694</anchor>
      <arglist>(json_sax &amp;&amp;) noexcept=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>parse_error</name>
      <anchorfile>structnlohmann_1_1json__sax_a60287e3bd85f489e04c83f7e3b76e613.html</anchorfile>
      <anchor>a60287e3bd85f489e04c83f7e3b76e613</anchor>
      <arglist>(std::size_t position, const std::string &amp;last_token, const detail::exception &amp;ex)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>start_array</name>
      <anchorfile>structnlohmann_1_1json__sax_a5c53878cf08d463eb4e7ca0270532572.html</anchorfile>
      <anchor>a5c53878cf08d463eb4e7ca0270532572</anchor>
      <arglist>(std::size_t elements)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>start_object</name>
      <anchorfile>structnlohmann_1_1json__sax_a0671528b0debb5a348169d61f0382a0f.html</anchorfile>
      <anchor>a0671528b0debb5a348169d61f0382a0f</anchor>
      <arglist>(std::size_t elements)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>string</name>
      <anchorfile>structnlohmann_1_1json__sax_a07eab82f6c82d606787eee9ad73d2bda.html</anchorfile>
      <anchor>a07eab82f6c82d606787eee9ad73d2bda</anchor>
      <arglist>(string_t &amp;val)=0</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>nlohmann::ordered_map</name>
    <filename>structnlohmann_1_1ordered__map.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <member kind="typedef">
      <type>std::vector&lt; std::pair&lt; const Key, T &gt;, Allocator &gt;</type>
      <name>Container</name>
      <anchorfile>structnlohmann_1_1ordered__map_a0cabe346c38a4f1ab1b8a396fbd2bbe2.html</anchorfile>
      <anchor>a0cabe346c38a4f1ab1b8a396fbd2bbe2</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>Key</type>
      <name>key_type</name>
      <anchorfile>structnlohmann_1_1ordered__map_a57095c6ed403f02e1bc2c240a13c9ed8.html</anchorfile>
      <anchor>a57095c6ed403f02e1bc2c240a13c9ed8</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>T</type>
      <name>mapped_type</name>
      <anchorfile>structnlohmann_1_1ordered__map_a1c9c1509ee714a9814b45a8030c84ec7.html</anchorfile>
      <anchor>a1c9c1509ee714a9814b45a8030c84ec7</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>typename std::enable_if&lt; std::is_convertible&lt; typename std::iterator_traits&lt; InputIt &gt;::iterator_category, std::input_iterator_tag &gt;::value &gt;::type</type>
      <name>require_input_iter</name>
      <anchorfile>structnlohmann_1_1ordered__map_a89cc338e8466e74baaa138664c79ee98.html</anchorfile>
      <anchor>a89cc338e8466e74baaa138664c79ee98</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ordered_map</name>
      <anchorfile>structnlohmann_1_1ordered__map_a87938c10b76510dac00412d2cb5fd1e4.html</anchorfile>
      <anchor>a87938c10b76510dac00412d2cb5fd1e4</anchor>
      <arglist>(const Allocator &amp;alloc=Allocator())</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ordered_map</name>
      <anchorfile>structnlohmann_1_1ordered__map_a9d25efb51325cc1be027b8ea00c1f8b8.html</anchorfile>
      <anchor>a9d25efb51325cc1be027b8ea00c1f8b8</anchor>
      <arglist>(It first, It last, const Allocator &amp;alloc=Allocator())</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ordered_map</name>
      <anchorfile>structnlohmann_1_1ordered__map_a0482ea79e7786367a2d9b5c789c091ce.html</anchorfile>
      <anchor>a0482ea79e7786367a2d9b5c789c091ce</anchor>
      <arglist>(std::initializer_list&lt; T &gt; init, const Allocator &amp;alloc=Allocator())</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>at</name>
      <anchorfile>structnlohmann_1_1ordered__map_ab7b4bb185fe7ea84f8f5f32fd230ff91.html</anchorfile>
      <anchor>ab7b4bb185fe7ea84f8f5f32fd230ff91</anchor>
      <arglist>(const Key &amp;key)</arglist>
    </member>
    <member kind="function">
      <type>const T &amp;</type>
      <name>at</name>
      <anchorfile>structnlohmann_1_1ordered__map_a8b7f27215180385b9b1e98adc4dd8ae7.html</anchorfile>
      <anchor>a8b7f27215180385b9b1e98adc4dd8ae7</anchor>
      <arglist>(const Key &amp;key) const</arglist>
    </member>
    <member kind="function">
      <type>size_type</type>
      <name>count</name>
      <anchorfile>structnlohmann_1_1ordered__map_aee2c188dcc802d6b28910f707a5e637b.html</anchorfile>
      <anchor>aee2c188dcc802d6b28910f707a5e637b</anchor>
      <arglist>(const Key &amp;key) const</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; iterator, bool &gt;</type>
      <name>emplace</name>
      <anchorfile>structnlohmann_1_1ordered__map_a38834c948b844033caa7d5c76fee5866.html</anchorfile>
      <anchor>a38834c948b844033caa7d5c76fee5866</anchor>
      <arglist>(const key_type &amp;key, T &amp;&amp;t)</arglist>
    </member>
    <member kind="function">
      <type>size_type</type>
      <name>erase</name>
      <anchorfile>structnlohmann_1_1ordered__map_a583c8976bbf0c137ff8e2439878f3058.html</anchorfile>
      <anchor>a583c8976bbf0c137ff8e2439878f3058</anchor>
      <arglist>(const Key &amp;key)</arglist>
    </member>
    <member kind="function">
      <type>iterator</type>
      <name>erase</name>
      <anchorfile>structnlohmann_1_1ordered__map_a26053569acb0a858d87482b2fa3d5dc5.html</anchorfile>
      <anchor>a26053569acb0a858d87482b2fa3d5dc5</anchor>
      <arglist>(iterator pos)</arglist>
    </member>
    <member kind="function">
      <type>iterator</type>
      <name>find</name>
      <anchorfile>structnlohmann_1_1ordered__map_a2486527ac56e07d58946ae9a93a46bc8.html</anchorfile>
      <anchor>a2486527ac56e07d58946ae9a93a46bc8</anchor>
      <arglist>(const Key &amp;key)</arglist>
    </member>
    <member kind="function">
      <type>const_iterator</type>
      <name>find</name>
      <anchorfile>structnlohmann_1_1ordered__map_a41e6e34fa8a90b96cbe5c71fec10d2ee.html</anchorfile>
      <anchor>a41e6e34fa8a90b96cbe5c71fec10d2ee</anchor>
      <arglist>(const Key &amp;key) const</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; iterator, bool &gt;</type>
      <name>insert</name>
      <anchorfile>structnlohmann_1_1ordered__map_a0241433138719e477a3cbb0c4cf0a243.html</anchorfile>
      <anchor>a0241433138719e477a3cbb0c4cf0a243</anchor>
      <arglist>(const value_type &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>insert</name>
      <anchorfile>structnlohmann_1_1ordered__map_a2c8509f72bc33307661f1a0ed5763f9e.html</anchorfile>
      <anchor>a2c8509f72bc33307661f1a0ed5763f9e</anchor>
      <arglist>(InputIt first, InputIt last)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; iterator, bool &gt;</type>
      <name>insert</name>
      <anchorfile>structnlohmann_1_1ordered__map_a48eceff729b80f3f4a023b737efccc5b.html</anchorfile>
      <anchor>a48eceff729b80f3f4a023b737efccc5b</anchor>
      <arglist>(value_type &amp;&amp;value)</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>operator[]</name>
      <anchorfile>structnlohmann_1_1ordered__map_ae7a1ca8c1e234837d137471f73ae6012.html</anchorfile>
      <anchor>ae7a1ca8c1e234837d137471f73ae6012</anchor>
      <arglist>(const Key &amp;key)</arglist>
    </member>
    <member kind="function">
      <type>const T &amp;</type>
      <name>operator[]</name>
      <anchorfile>structnlohmann_1_1ordered__map_a676082659d575e29bdb312bcde53023a.html</anchorfile>
      <anchor>a676082659d575e29bdb312bcde53023a</anchor>
      <arglist>(const Key &amp;key) const</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>nlohmann</name>
    <filename>namespacenlohmann.html</filename>
    <namespace>nlohmann::anonymous_namespace{json.hpp}</namespace>
    <class kind="struct">nlohmann::adl_serializer</class>
    <class kind="class">nlohmann::basic_json</class>
    <class kind="class">nlohmann::byte_container_with_subtype</class>
    <class kind="class">nlohmann::json_pointer</class>
    <class kind="struct">nlohmann::json_sax</class>
    <class kind="struct">nlohmann::ordered_map</class>
    <member kind="typedef">
      <type>basic_json&lt;&gt;</type>
      <name>json</name>
      <anchorfile>namespacenlohmann_a2bfd99e845a2e5cd90aeaf1b1431f474.html</anchorfile>
      <anchor>a2bfd99e845a2e5cd90aeaf1b1431f474</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>basic_json&lt; nlohmann::ordered_map &gt;</type>
      <name>ordered_json</name>
      <anchorfile>namespacenlohmann_ad53cef358adfa7f07cea23eb1e28b9ea.html</anchorfile>
      <anchor>ad53cef358adfa7f07cea23eb1e28b9ea</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>NLOHMANN_BASIC_JSON_TPL_DECLARATION std::string</type>
      <name>to_string</name>
      <anchorfile>namespacenlohmann_a6ce645a0b8717757e096a5b5773b7a16.html</anchorfile>
      <anchor>a6ce645a0b8717757e096a5b5773b7a16</anchor>
      <arglist>(const NLOHMANN_BASIC_JSON_TPL &amp;j)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>nlohmann::anonymous_namespace{json.hpp}</name>
    <filename>namespacenlohmann_1_1anonymous__namespace_02json_8hpp_03.html</filename>
    <member kind="variable">
      <type>constexpr const auto &amp;</type>
      <name>from_json</name>
      <anchorfile>namespacenlohmann_1_1anonymous__namespace_02json_8hpp_03_a69afe041fa2aeac2239b65ae88b64af8.html</anchorfile>
      <anchor>a69afe041fa2aeac2239b65ae88b64af8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>constexpr const auto &amp;</type>
      <name>to_json</name>
      <anchorfile>namespacenlohmann_1_1anonymous__namespace_02json_8hpp_03_a455d0daa616e67bbb74d81cf3ba15e79.html</anchorfile>
      <anchor>a455d0daa616e67bbb74d81cf3ba15e79</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="page">
    <name>index</name>
    <title>JSON for Modern C++</title>
    <filename>index.html</filename>
    <docanchor file="index.html">md_index</docanchor>
  </compound>
</tagfile>
