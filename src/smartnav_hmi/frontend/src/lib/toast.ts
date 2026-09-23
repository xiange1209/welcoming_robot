import { toast as sonnerToast } from "sonner";

export type ToastMessage = string | undefined | null;
export type ToastType = "success" | "error" | "warning" | "info";

export function toast(message: ToastMessage, type?: ToastType) {
  const text = (message ?? "").trim();
  if (!text) return;

  if (type) {
    sonnerToast[type](text);
  } else {
    sonnerToast(text);
  }
}

toast.success = (msg: ToastMessage) => toast(msg, "success");
toast.warning = (msg: ToastMessage) => toast(msg, "warning");
toast.error = (msg: ToastMessage) => toast(msg, "error");
toast.info = (msg: ToastMessage) => toast(msg, "info");
