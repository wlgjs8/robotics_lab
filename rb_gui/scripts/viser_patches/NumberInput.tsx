import * as React from "react";
import { GuiComponentContext } from "../ControlPanel/GuiComponentContext";
import { GuiNumberMessage } from "../WebsocketMessages";
import { ViserInputComponent } from "./common";
import { NumberInput } from "@mantine/core";

// ---------------------------------------------------------------------------
// robotics_lab patch (Enter-to-commit, focus-aware mirror)
//
// Upstream viser streams every keystroke to the server (Mantine onChange fires
// per character), so typing a multi-digit target into a number field jogged the
// robot through each intermediate value. We instead:
//   * keep the in-progress edit in local React state (`draft`),
//   * commit to the server ONLY on Enter (the sole commit path; blur/Escape
//     discard the edit),
//   * while the input is focused, ignore incoming server `value` updates so a
//     live state-mirror cannot clobber what the operator is typing; while
//     unfocused, mirror the server `value` as usual.
//
// Re-apply after any viser reinstall/upgrade via
//   robotics_lab/rb_gui/scripts/patch_viser_number_enter.sh
// ---------------------------------------------------------------------------
export default function NumberInputComponent({
  uuid,
  value,
  props: { visible, label, hint, disabled, precision, min, max, step },
}: GuiNumberMessage) {
  const { setValue } = React.useContext(GuiComponentContext)!;
  const [draft, setDraft] = React.useState<number | string>(value);
  const focusedRef = React.useRef(false);

  // Mirror the server value into the field only while it is NOT being edited.
  React.useEffect(() => {
    if (!focusedRef.current) {
      setDraft(value);
    }
  }, [value]);

  if (!visible) return null;

  const commit = () => {
    if (draft !== "" && draft !== value) {
      setValue(uuid, draft);
    }
  };

  return (
    <ViserInputComponent {...{ uuid, hint, label }}>
      <NumberInput
        id={uuid}
        value={draft}
        // This was renamed in Mantine v7.
        decimalScale={precision}
        min={min ?? undefined}
        max={max ?? undefined}
        step={step}
        size="xs"
        onChange={(newValue) => {
          // Stage the edit locally; do NOT push to the server until Enter.
          newValue !== "" && setDraft(newValue);
        }}
        onFocus={() => {
          focusedRef.current = true;
        }}
        onBlur={() => {
          focusedRef.current = false;
          // Discard any uncommitted edit — Enter is the only commit path.
          setDraft(value);
        }}
        onKeyDown={(e) => {
          if (e.key === "Enter") {
            commit();
          } else if (e.key === "Escape") {
            setDraft(value);
          }
        }}
        styles={{
          input: {
            minHeight: "1.625rem",
            height: "1.625rem",
          },
          controls: {
            height: "1.625em",
            width: "0.825em",
          },
        }}
        disabled={disabled}
        stepHoldDelay={500}
        stepHoldInterval={(t) => Math.max(1000 / t ** 2, 25)}
      />
    </ViserInputComponent>
  );
}
