#pragma once

#include <imgui.h>
#include <array>
#include <atomic>

namespace lidar_viewer::ui
{
enum class Theme {Dark, Light};

// Appearance belongs to the application, including all of its workspaces.
inline std::atomic<Theme> & ApplicationTheme() {static std::atomic<Theme> theme{Theme::Dark};return theme;}
inline bool IsLight() {return ApplicationTheme().load(std::memory_order_relaxed)==Theme::Light;}
inline const char * ThemeName() {return IsLight() ? "light" : "dark";}
inline ImVec4 Tone(ImVec4 dark,ImVec4 light) {return IsLight() ? light : dark;}
inline ImVec4 ErrorColor() {return Tone({1,.56f,.48f,1},{.73f,.16f,.15f,1});}
inline ImVec4 WarningColor() {return Tone({1,.74f,.4f,1},{.58f,.32f,.04f,1});}
inline ImVec4 SuccessColor() {return Tone({.42f,.85f,.65f,1},{.08f,.44f,.28f,1});}
inline ImVec4 InfoColor() {return Tone({.44f,.76f,1,1},{.08f,.36f,.59f,1});}
inline ImVec4 SelectionColor() {return Tone({.13f,.32f,.40f,1},{.80f,.89f,.95f,1});}
inline ImVec4 OverlayBackground() {return Tone({.08f,.09f,.11f,.96f},{.97f,.98f,1,.96f});}
inline ImVec4 DetectionLabelBackground() {return Tone({.035f,.05f,.07f,1},{1,1,1,1});}
inline ImVec4 DetectionLabelText() {return Tone({.96f,.98f,1,1},{.08f,.12f,.17f,1});}
inline ImVec4 AppBackground() {return Tone({.061f,.066f,.079f,1},{.94f,.95f,.97f,1});}
inline ImVec4 CanvasBackground() {return Tone({.042f,.047f,.059f,1},{1,1,1,1});}
inline ImVec4 GridColor() {return Tone({.42f,.48f,.56f,.38f},{.48f,.54f,.62f,.22f});}
inline ImVec4 PlainPointColor() {return Tone({.82f,.88f,.94f,1},{.22f,.27f,.34f,1});}
inline ImVec4 BackgroundPointColor() {return Tone({.68f,.72f,.78f,1},{.52f,.56f,.62f,1});}
inline ImVec4 UnlabeledPointColor() {return Tone({.28f,.32f,.38f,1},{.72f,.75f,.79f,1});}

inline const std::array<ImVec4,8> & LabelColors()
{
  static const std::array<ImVec4,8> dark{{
    {1,.59f,.31f,1},{.40f,.61f,1,1},{.39f,.84f,.55f,1},{.70f,.54f,1,1},
    {.97f,.49f,.73f,1},{.96f,.82f,.35f,1},{.33f,.80f,.87f,1},{.98f,.44f,.44f,1}}};
  static const std::array<ImVec4,8> light{{
    {.76f,.34f,.07f,1},{.12f,.35f,.73f,1},{.09f,.45f,.24f,1},{.48f,.28f,.70f,1},
    {.68f,.21f,.43f,1},{.53f,.40f,.03f,1},{.04f,.43f,.49f,1},{.74f,.16f,.18f,1}}};
  return IsLight() ? light : dark;
}

inline void ApplyThemeColors(ImGuiStyle & style)
{
  if(IsLight()) {
    ImGui::StyleColorsLight(&style);
    style.Colors[ImGuiCol_Text]={.12f,.17f,.23f,1};
    style.Colors[ImGuiCol_TextDisabled]={.38f,.43f,.50f,1};
    style.Colors[ImGuiCol_WindowBg]={.98f,.985f,.995f,1};
    style.Colors[ImGuiCol_PopupBg]={1,1,1,.99f};
    style.Colors[ImGuiCol_Border]={.82f,.85f,.89f,1};
    style.Colors[ImGuiCol_Separator]=style.Colors[ImGuiCol_Border];
    style.Colors[ImGuiCol_FrameBg]={.92f,.94f,.97f,1};
    style.Colors[ImGuiCol_FrameBgHovered]={.83f,.89f,.94f,1};
    style.Colors[ImGuiCol_FrameBgActive]={.75f,.85f,.93f,1};
    style.Colors[ImGuiCol_Button]={.90f,.92f,.95f,1};
    style.Colors[ImGuiCol_ButtonHovered]={.75f,.85f,.92f,1};
    style.Colors[ImGuiCol_ButtonActive]={.64f,.77f,.87f,1};
    style.Colors[ImGuiCol_CheckMark]={.05f,.37f,.56f,1};
    style.Colors[ImGuiCol_SliderGrab]={.18f,.44f,.63f,1};
    style.Colors[ImGuiCol_SliderGrabActive]={.05f,.31f,.48f,1};
    style.Colors[ImGuiCol_Header]={.84f,.90f,.95f,1};
    style.Colors[ImGuiCol_HeaderHovered]={.73f,.84f,.93f,1};
    style.Colors[ImGuiCol_HeaderActive]={.65f,.78f,.88f,1};
    style.Colors[ImGuiCol_Tab]={.90f,.93f,.97f,1};
    style.Colors[ImGuiCol_TabHovered]={.74f,.85f,.93f,1};
    style.Colors[ImGuiCol_TabSelected]={.80f,.89f,.95f,1};
    style.Colors[ImGuiCol_TabSelectedOverline]={.08f,.38f,.57f,1};
    style.Colors[ImGuiCol_NavCursor]=style.Colors[ImGuiCol_CheckMark];
  } else {
    ImGui::StyleColorsDark(&style);
    style.Colors[ImGuiCol_Text]={.90f,.93f,.97f,1};
    style.Colors[ImGuiCol_TextDisabled]={.62f,.66f,.73f,1};
    style.Colors[ImGuiCol_WindowBg]={.086f,.094f,.114f,1};
    style.Colors[ImGuiCol_PopupBg]={.107f,.117f,.142f,1};
    style.Colors[ImGuiCol_ModalWindowDimBg]={.025f,.028f,.036f,.55f};
    style.Colors[ImGuiCol_Border]={.174f,.188f,.222f,1};
    style.Colors[ImGuiCol_Separator]=style.Colors[ImGuiCol_Border];
    style.Colors[ImGuiCol_FrameBg]={.123f,.137f,.165f,1};
    style.Colors[ImGuiCol_FrameBgHovered]={.169f,.20f,.245f,1};
    style.Colors[ImGuiCol_FrameBgActive]={.182f,.252f,.321f,1};
    style.Colors[ImGuiCol_Button]={.139f,.154f,.184f,1};
    style.Colors[ImGuiCol_ButtonHovered]={.19f,.23f,.288f,1};
    style.Colors[ImGuiCol_ButtonActive]={.21f,.31f,.40f,1};
    style.Colors[ImGuiCol_CheckMark]={.31f,.77f,.91f,1};
    style.Colors[ImGuiCol_SliderGrab]={.31f,.77f,.91f,1};
    style.Colors[ImGuiCol_SliderGrabActive]={.49f,.88f,1,1};
    style.Colors[ImGuiCol_Header]={.144f,.161f,.19f,1};
    style.Colors[ImGuiCol_HeaderHovered]={.19f,.26f,.334f,1};
    style.Colors[ImGuiCol_Tab]={.105f,.116f,.14f,1};
    style.Colors[ImGuiCol_TabHovered]={.167f,.228f,.29f,1};
    style.Colors[ImGuiCol_TabSelected]={.13f,.32f,.40f,1};
    style.Colors[ImGuiCol_TabSelectedOverline]={.31f,.77f,.91f,1};
    style.Colors[ImGuiCol_NavCursor]=style.Colors[ImGuiCol_CheckMark];
  }
}
}  // namespace lidar_viewer::ui
