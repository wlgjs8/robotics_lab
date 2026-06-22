import * as React from "react";
import { useDisclosure } from "@mantine/hooks";
import { GuiFolderMessage } from "../WebsocketMessages";
import { IconChevronDown, IconChevronUp } from "@tabler/icons-react";
import { Box, Collapse, Paper } from "@mantine/core";
import { GuiComponentContext } from "../ControlPanel/GuiComponentContext";
import { ViewerContext } from "../ViewerContext";
import { folderLabel, folderToggleIcon, folderWrapper } from "./Folder.css";
import { shallowObjectKeysEqual } from "../utils/shallowObjectKeysEqual";

export default function FolderComponent({
  uuid,
  props: { label, visible, expand_by_default },
  nextGuiUuid,
}: GuiFolderMessage & { nextGuiUuid: string | null }) {
  const viewer = React.useContext(ViewerContext)!;
  const [opened, { toggle }] = useDisclosure(expand_by_default);
  const guiIdSet = viewer.useGui(
    (state) => state.guiUuidSetFromContainerUuid[uuid],
    shallowObjectKeysEqual,
  );
  const guiContext = React.useContext(GuiComponentContext)!;
  const isEmpty = guiIdSet === undefined || Object.keys(guiIdSet).length === 0;
  const nextGuiType = viewer.useGuiConfig(nextGuiUuid ?? "", (conf) =>
    nextGuiUuid == null ? null : (conf?.type ?? null),
  );

  if (!visible) return null;

  // No label: render children only, no header/border/collapse. Use
  // `unwrapped` so we don't introduce extra padding above the first child.
  if (label === null) {
    return (
      <GuiComponentContext.Provider
        value={{
          ...guiContext,
          folderDepth: guiContext.folderDepth + 1,
        }}
      >
        <guiContext.GuiContainer containerUuid={uuid} unwrapped />
      </GuiComponentContext.Provider>
    );
  }

  const ToggleIcon = opened ? IconChevronUp : IconChevronDown;

  // robotics_lab patch: a `⟦cols=N⟧` marker in the label lays this folder's
  // direct children out in an N-column CSS grid (instead of the default vertical
  // stack), so e.g. left/right arm sub-folders can sit side by side. The marker
  // is stripped from the visible label. See patch_viser_number_enter.sh.
  let displayLabel = label;
  let gridCols: number | null = null;
  if (typeof label === "string") {
    const m = label.match(/⟦cols=(\d+)⟧\s*$/);
    if (m) {
      gridCols = Math.max(1, parseInt(m[1], 10));
      displayLabel = label.replace(/⟦cols=(\d+)⟧\s*$/, "").trimEnd();
    }
  }
  const gridStyle = gridCols
    ? {
        display: "grid",
        gridTemplateColumns: `repeat(${gridCols}, minmax(0, 1fr))`,
        gap: "0.25em",
        alignItems: "start" as const,
      }
    : undefined;

  return (
    <Paper
      withBorder
      className={folderWrapper}
      mb={nextGuiType === "GuiFolderMessage" || nextGuiType === "GuiFormMessage" ? "md" : undefined}
    >
      <Paper
        className={folderLabel}
        style={{
          cursor: isEmpty ? undefined : "pointer",
        }}
        onClick={toggle}
      >
        {displayLabel}
        <ToggleIcon
          className={folderToggleIcon}
          style={{
            display: isEmpty ? "none" : undefined,
          }}
        />
      </Paper>
      <Collapse in={opened && !isEmpty}>
        <Box pt="0.2em" style={gridStyle}>
          <GuiComponentContext.Provider
            value={{
              ...guiContext,
              folderDepth: guiContext.folderDepth + 1,
            }}
          >
            <guiContext.GuiContainer containerUuid={uuid} unwrapped={gridCols != null} />
          </GuiComponentContext.Provider>
        </Box>
      </Collapse>
      <Collapse in={!(opened && !isEmpty)}>
        <Box p="xs"></Box>
      </Collapse>
    </Paper>
  );
}
